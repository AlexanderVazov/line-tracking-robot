/*
 * ESP32-CAM Line Detector Preview + Motor Control
 * Hardware: AI-Thinker ESP32-CAM + L298N Motor Driver
 *
 * Open http://<device-IP> in a browser to see:
 *   - Live camera feed with detection overlay drawn in the browser:
 *       · Semi-transparent dimming outside the active scan band
 *       · Dashed white lines marking the scan band boundaries
 *       · Dotted grey centre reference line
 *       · Cyan vertical line at the detected line position
 *   - Status badge: LINE DETECTED / No line
 *   - Detected X position and error from centre
 *   - Which zone the error falls into (STRAIGHT / TURN / SHARP TURN)
 *
 * L298N Wiring:
 *   ENA  -> GPIO 2   (Left motor PWM)
 *   IN1  -> GPIO 14  (Left motor forward)
 *   IN2  -> GPIO 15  (Left motor reverse)
 *   ENB  -> GPIO 3   (Right motor PWM)
 *   IN3  -> GPIO 13  (Right motor forward)
 *   IN4  -> GPIO 12  (Right motor reverse)
 *
 * LEDC channel assignments:
 *   Channel 0 → Left motor ENA  (GPIO 2)
 *   Channel 1 → Right motor ENB (GPIO 3)
 *   Channel 2 → Camera XCLK    (reserved by esp_camera)
 *
 * Notes:
 *   - GPIO 3 is U0RXD — Serial.end() is called before motor setup
 *   - Flash LED (GPIO 4): ON = line detected, OFF = no line
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>

// ── WiFi ──────────────────────────────────────────────────────────────────────
#define WIFI_SSID  "SSID"
#define WIFI_PASS  "PASSWORD"

// ── Camera pin map (AI-Thinker) ───────────────────────────────────────────────
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ── Flash LED ─────────────────────────────────────────────────────────────────
#define LED_PIN  4

// ── Motor driver pins ─────────────────────────────────────────────────────────
#define ENA_PIN  2    // Left  motor PWM
#define IN1_PIN  14   // Left  motor +
#define IN2_PIN  15   // Left  motor -
#define ENB_PIN  3    // Right motor PWM (U0RXD — Serial unavailable after Serial.end())
#define IN3_PIN  13   // Right motor +
#define IN4_PIN  12   // Right motor -

#define ENA_CHAN  0
#define ENB_CHAN  1
#define PWM_FREQ  1000
#define PWM_RES   8

// ── Image ─────────────────────────────────────────────────────────────────────
#define IMG_WIDTH   160
#define IMG_HEIGHT  120

// ── Detection ─────────────────────────────────────────────────────────────────
#define SCAN_TOP            40
#define SCAN_BOTTOM        115
#define SCAN_ROWS          (SCAN_BOTTOM - SCAN_TOP + 1)
#define CONTRAST_THRESHOLD  40
#define MIN_BACKGROUND     100   // avg pixel brightness required — rejects wood floor

// ── Zone-based control ────────────────────────────────────────────────────────
#define ZONE_CENTER   30   // wider dead-band reduces oscillation from camera swing
#define ZONE_FAR      55

#define BASE_SPEED    110
#define INNER_SLOW     25
#define INNER_REVERSE -50

#define FORWARD_MS    25
#define TURN_SHORT_MS 25
#define TURN_LONG_MS  35   // shorter — prevents camera swing overshooting the line
#define LOST_TIMEOUT_MS 800

// ── Recent memory (line recovery) ─────────────────────────────────────────────
// Recent error samples are kept while the line is visible. When it disappears
// we look at this history to decide which way the line was heading and use
// that as the search direction — instead of just the last single frame.
#define MEM_SAMPLES   16   // ~half-second of history at typical frame rate

// ── Static-friction kick ─────────────────────────────────────────────────────
// Low PWM (e.g. INNER_SLOW=25) doesn't break static friction from a dead stop —
// the motors just buzz. On any 0 → nonzero transition we hold KICK_SPEED for
// KICK_MS to spin the wheels up, then settle to the commanded speed.
#define KICK_SPEED 220
#define KICK_MS    80

// Camera is 15 cm ahead of the wheel axles.  On a curve the apparent pixel error
// overstates the correction the axles actually need, causing oscillation.
// Scale the error down before zone decisions to compensate.
#define CAM_SCALE_NUM  2
#define CAM_SCALE_DEN  3

// ─────────────────────────────────────────────────────────────────────────────

static bool          gLineFound = false;
static int           gLineX     = -1;
static int           gError     = 0;

static int           lastError   = 0;
static bool          lineLost    = false;
static unsigned long lostAt      = 0;
static bool          serverReady = false;

// Rolling average — smooths frame-to-frame noise before zone decisions
#define ERR_SAMPLES 4
static int  errBuf[ERR_SAMPLES] = {};
static int  errIdx  = 0;
static bool errFull = false;

// Recent-memory buffer — fills while the line is found, frozen while lost
static int  memBuf[MEM_SAMPLES] = {};
static int  memIdx         = 0;
static int  memCount       = 0;
static int  memErrorAtLoss = 0;   // captured the moment the line disappeared

static WebServer server(80);

// ── Motors ────────────────────────────────────────────────────────────────────

// Last commanded speed per wheel — used to detect 0 → nonzero transitions
// so the static-friction kick fires at the right moment.
static int lastLeftSpeed  = 0;
static int lastRightSpeed = 0;

void driveLeft(int speed) {
    bool kick = (speed != 0) && (lastLeftSpeed == 0);
    if (speed >= 0) { digitalWrite(IN1_PIN, HIGH); digitalWrite(IN2_PIN, LOW);  }
    else            { digitalWrite(IN1_PIN, LOW);  digitalWrite(IN2_PIN, HIGH); }
    if (kick) {
        ledcWrite(ENA_PIN, KICK_SPEED);
        delay(KICK_MS);
    }
    ledcWrite(ENA_PIN, constrain(abs(speed), 0, 255));
    lastLeftSpeed = speed;
}

void driveRight(int speed) {
    bool kick = (speed != 0) && (lastRightSpeed == 0);
    if (speed >= 0) { digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);  }
    else            { digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH); }
    if (kick) {
        ledcWrite(ENB_PIN, KICK_SPEED);
        delay(KICK_MS);
    }
    ledcWrite(ENB_PIN, constrain(abs(speed), 0, 255));
    lastRightSpeed = speed;
}

void stopMotors() {
    digitalWrite(IN1_PIN, LOW); digitalWrite(IN2_PIN, LOW);
    digitalWrite(IN3_PIN, LOW); digitalWrite(IN4_PIN, LOW);
    ledcWrite(ENA_PIN, 0);
    ledcWrite(ENB_PIN, 0);
    lastLeftSpeed  = 0;
    lastRightSpeed = 0;
}

void setupMotors() {
    ledcAttachChannel(ENA_PIN, PWM_FREQ, PWM_RES, ENA_CHAN);
    ledcAttachChannel(ENB_PIN, PWM_FREQ, PWM_RES, ENB_CHAN);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    stopMotors();
}

// Drive for ms milliseconds while keeping the web server alive.
void driveFor(unsigned long ms) {
    unsigned long end = millis() + ms;
    while (millis() < end) {
        server.handleClient();
        delay(1);
    }
}

// ── Camera ────────────────────────────────────────────────────────────────────

bool setupCamera() {
    camera_config_t cfg;
    cfg.ledc_channel  = LEDC_CHANNEL_2;
    cfg.ledc_timer    = LEDC_TIMER_2;
    cfg.pin_d0        = Y2_GPIO_NUM;
    cfg.pin_d1        = Y3_GPIO_NUM;
    cfg.pin_d2        = Y4_GPIO_NUM;
    cfg.pin_d3        = Y5_GPIO_NUM;
    cfg.pin_d4        = Y6_GPIO_NUM;
    cfg.pin_d5        = Y7_GPIO_NUM;
    cfg.pin_d6        = Y8_GPIO_NUM;
    cfg.pin_d7        = Y9_GPIO_NUM;
    cfg.pin_xclk      = XCLK_GPIO_NUM;
    cfg.pin_pclk      = PCLK_GPIO_NUM;
    cfg.pin_vsync     = VSYNC_GPIO_NUM;
    cfg.pin_href      = HREF_GPIO_NUM;
    cfg.pin_sscb_sda  = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl  = SIOC_GPIO_NUM;
    cfg.pin_pwdn      = PWDN_GPIO_NUM;
    cfg.pin_reset     = RESET_GPIO_NUM;
    cfg.xclk_freq_hz  = 20000000;
    cfg.pixel_format  = PIXFORMAT_GRAYSCALE;
    cfg.frame_size    = FRAMESIZE_QQVGA;
    cfg.jpeg_quality  = 12;
    if (psramFound()) {
        cfg.fb_count    = 2;
        cfg.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        cfg.fb_count    = 1;
        cfg.fb_location = CAMERA_FB_IN_DRAM;
    }

    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("Camera init failed");
        return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    s->set_contrast(s, 2);
    s->set_brightness(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 0);

    Serial.println("Camera OK");
    return true;
}

// ── Detection ─────────────────────────────────────────────────────────────────

int detectLine(const uint8_t* pixels) {
    long colSum[IMG_WIDTH] = {};
    for (int row = SCAN_TOP; row <= SCAN_BOTTOM; row++) {
        const int offset = row * IMG_WIDTH;
        for (int x = 0; x < IMG_WIDTH; x++) colSum[x] += pixels[offset + x];
    }

    long total = 0;
    for (int x = 0; x < IMG_WIDTH; x++) total += colSum[x];
    long mean = total / IMG_WIDTH;

    // Reject if the background isn't bright enough to be white paper
    if (mean / SCAN_ROWS < MIN_BACKGROUND) return -1;

    // Find the darkest column (black tape on white — only look darker than mean)
    long maxDev = 0;
    int  peakCol = IMG_WIDTH / 2;
    for (int x = 0; x < IMG_WIDTH; x++) {
        long dev = mean - colSum[x];   // positive only for dark columns
        if (dev > maxDev) { maxDev = dev; peakCol = x; }
    }

    long minDev = (long)CONTRAST_THRESHOLD * SCAN_ROWS;
    if (maxDev < minDev) return -1;

    // Centroid of all columns that are similarly dark
    long wsum = 0;
    int  cnt  = 0;
    for (int x = 0; x < IMG_WIDTH; x++) {
        if (mean - colSum[x] >= minDev) { wsum += x; cnt++; }
    }

    return cnt > 0 ? (int)(wsum / cnt) : peakCol;
}

// ── Web page ──────────────────────────────────────────────────────────────────

static const char INDEX_HTML[] = R"html(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Line Detector Preview</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{background:#111827;color:#f9fafb;font-family:system-ui,sans-serif;
         display:flex;flex-direction:column;align-items:center;padding:24px 12px}
    h1{font-size:1.3rem;margin-bottom:16px;letter-spacing:.05em}
    #cam{display:block;width:320px;height:240px;border:2px solid #374151;
         border-radius:6px;background:#1f2937}
    #badge{margin-top:14px;padding:8px 28px;border-radius:9999px;
           font-size:1rem;font-weight:600;transition:background .15s}
    .on {background:#16a34a;color:#fff}
    .off{background:#374151;color:#9ca3af}
    #details{margin-top:8px;font-size:.78rem;color:#9ca3af;text-align:center;line-height:1.7}
    #zone{margin-top:6px;font-size:.85rem;font-weight:600;color:#fbbf24}
    #legend{margin-top:14px;font-size:.72rem;color:#6b7280;line-height:2;text-align:left}
    span.swatch{display:inline-block;width:28px;height:2px;vertical-align:middle;margin-right:4px}
  </style>
</head>
<body>
  <h1>Line Detector Preview</h1>
  <canvas id="cam" width="320" height="240"></canvas>
  <div id="badge" class="off">No line</div>
  <div id="details">x: — &nbsp;|&nbsp; error: —</div>
  <div id="zone">—</div>
  <div id="legend">
    <span class="swatch" style="border-top:2px dashed white"></span>Dashed white = scan band boundaries<br>
    <span class="swatch" style="border-top:2px dotted #aaa"></span>Dotted grey = frame centre (x=80)<br>
    <span class="swatch" style="background:cyan"></span>Cyan = detected line position<br>
    Dimmed areas = outside active scan band
  </div>

  <script>
    var canvas  = document.getElementById('cam');
    var ctx     = canvas.getContext('2d');
    var badgeEl = document.getElementById('badge');
    var detailEl= document.getElementById('details');
    var zoneEl  = document.getElementById('zone');

    var SX = canvas.width  / 160;
    var SY = canvas.height / 120;

    var det = {line:false, x:-1, error:0, zone:'—', scanTop:40, scanBottom:115, centerX:80};

    setInterval(function(){
      fetch('/status')
        .then(function(r){return r.json();})
        .then(function(d){
          det = d;
          if(d.line){
            badgeEl.textContent='LINE DETECTED'; badgeEl.className='on';
            detailEl.textContent='x: '+d.x+'  |  error: '+d.error;
            zoneEl.textContent=d.zone;
          } else {
            badgeEl.textContent='No line'; badgeEl.className='off';
            detailEl.textContent='x: —  |  error: —';
            zoneEl.textContent='—';
          }
        }).catch(function(){});
    }, 200);

    function drawOverlay(){
      var st = det.scanTop    * SY;
      var sb = det.scanBottom * SY;
      var cx = det.centerX    * SX;

      ctx.fillStyle='rgba(0,0,0,0.55)';
      ctx.fillRect(0, 0, canvas.width, st);
      ctx.fillRect(0, sb, canvas.width, canvas.height - sb);

      ctx.save();
      ctx.strokeStyle='white';
      ctx.lineWidth=1.5;
      ctx.setLineDash([6,4]);
      ctx.beginPath(); ctx.moveTo(0,st); ctx.lineTo(canvas.width,st); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0,sb); ctx.lineTo(canvas.width,sb); ctx.stroke();

      ctx.strokeStyle='rgba(200,200,200,0.6)';
      ctx.lineWidth=1;
      ctx.setLineDash([3,4]);
      ctx.beginPath(); ctx.moveTo(cx,st); ctx.lineTo(cx,sb); ctx.stroke();
      ctx.restore();

      if(det.line && det.x >= 0){
        var lx = det.x * SX;
        ctx.save();
        ctx.strokeStyle='cyan';
        ctx.lineWidth=2;
        ctx.beginPath(); ctx.moveTo(lx,st); ctx.lineTo(lx,sb); ctx.stroke();
        ctx.restore();
      }
    }

    function nextFrame(){
      var img = new Image();
      img.onload = function(){
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        drawOverlay();
        setTimeout(nextFrame, 80);
      };
      img.onerror = function(){ setTimeout(nextFrame, 500); };
      img.src = '/snapshot?' + Date.now();
    }
    nextFrame();
  </script>
</body>
</html>
)html";

// ── Web handlers ──────────────────────────────────────────────────────────────

void handleRoot() {
    server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
    int absErr = abs(gError);
    const char* zone = "—";
    if (gLineFound) {
        if      (absErr <= ZONE_CENTER) zone = "STRAIGHT";
        else if (absErr <= ZONE_FAR)    zone = "TURN";
        else                            zone = "SHARP TURN";
    }
    String json = "{\"line\":";
    json += gLineFound ? "true" : "false";
    json += ",\"x\":"       + String(gLineX);
    json += ",\"error\":"   + String(gError);
    json += ",\"zone\":\""  + String(zone) + "\"";
    json += ",\"scanTop\":"    + String(SCAN_TOP);
    json += ",\"scanBottom\":" + String(SCAN_BOTTOM);
    json += ",\"centerX\":"    + String(IMG_WIDTH / 2);
    json += "}";
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
}

void handleSnapshot() {
    // Detection runs in loop() — here we just serve the current frame.
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { server.send(503, "text/plain", "Camera error"); return; }

    uint8_t* jpg    = nullptr;
    size_t   jpgLen = 0;
    bool ok = frame2jpg(fb, 20, &jpg, &jpgLen);
    esp_camera_fb_return(fb);

    if (!ok || !jpg) { server.send(500, "text/plain", "JPEG conversion failed"); return; }

    server.sendHeader("Cache-Control", "no-cache, no-store");
    server.setContentLength(jpgLen);
    server.send(200, "image/jpeg", "");
    server.client().write((const uint8_t*)jpg, jpgLen);
    free(jpg);
}

// ── WiFi ──────────────────────────────────────────────────────────────────────

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500); Serial.print("."); tries++;
    }
    if (WiFi.status() == WL_CONNECTED)
        Serial.printf("\nConnected — http://%s\n", WiFi.localIP().toString().c_str());
    else
        Serial.println("\nWiFi failed");
}

// ─────────────────────────────────────────────────────────────────────────────

static bool cameraReady = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    for (int attempt = 1; attempt <= 5 && !cameraReady; attempt++) {
        Serial.printf("Camera init attempt %d/5\n", attempt);
        cameraReady = setupCamera();
        if (!cameraReady) { esp_camera_deinit(); delay(500); }
    }

    if (!cameraReady) {
        Serial.println("Camera failed — check wiring");
        while (true) { digitalWrite(LED_PIN, HIGH); delay(80); digitalWrite(LED_PIN, LOW); delay(80); }
    }

    setupWiFi();

    if (WiFi.status() == WL_CONNECTED) {
        server.on("/",         HTTP_GET, handleRoot);
        server.on("/status",   HTTP_GET, handleStatus);
        server.on("/snapshot", HTTP_GET, handleSnapshot);
        server.begin();
        serverReady = true;
        Serial.println("Web server started");
    } else {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

    Serial.println("Starting in 1s");
    Serial.flush();
    Serial.end();   // release GPIO 3 (U0RXD) so LEDC can drive ENB (right motor)

    setupMotors();
    delay(1000);
}

// Weighted average of the recent-memory buffer — newer samples count more.
// Captures the direction the line was heading just before it disappeared.
int recoveryError() {
    if (memCount == 0) return lastError;
    long sum = 0, weights = 0;
    for (int i = 0; i < memCount; i++) {
        int idx = (memIdx - 1 - i + MEM_SAMPLES) % MEM_SAMPLES;
        int w   = memCount - i;   // newest sample gets the largest weight
        sum     += (long)memBuf[idx] * w;
        weights += w;
    }
    return (int)(sum / weights);
}

void loop() {
    if (!cameraReady) { server.handleClient(); delay(100); return; }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { stopMotors(); server.handleClient(); delay(30); return; }

    int lineX = detectLine(fb->buf);
    esp_camera_fb_return(fb);

    // Update globals read by /status and /snapshot handlers
    gLineFound = (lineX >= 0);
    gLineX     = lineX;
    gError     = gLineFound ? (IMG_WIDTH / 2) - lineX : 0;

    // ── Line lost ────────────────────────────────────────────────────────────
    if (lineX < 0) {
        digitalWrite(LED_PIN, LOW);
        if (!lineLost) {
            lineLost = true; lostAt = millis();
            errFull = false; errIdx = 0; memset(errBuf, 0, sizeof(errBuf));
            // Snapshot the recovery direction. memBuf itself is NOT cleared —
            // it stays frozen at the values seen just before the line vanished.
            memErrorAtLoss = recoveryError();
        }

        if (millis() - lostAt > LOST_TIMEOUT_MS) {
            stopMotors();
        } else {
            // Pivot in place at full speed toward where memory says the line
            // was heading. Direction matches this codebase's existing convention.
            int t = BASE_SPEED;
            if (memErrorAtLoss >= 0) { driveLeft(t);  driveRight(-t); }
            else                     { driveLeft(-t); driveRight(t);  }
        }
        if (serverReady) server.handleClient();
        delay(20);
        return;
    }

    // ── Line found — zone-based control ──────────────────────────────────────
    digitalWrite(LED_PIN, HIGH);
    lineLost = false;

    int error = (IMG_WIDTH / 2) - lineX;
    lastError = error;

    // Push into recovery memory — used to bias direction if the line is lost
    memBuf[memIdx] = error;
    memIdx = (memIdx + 1) % MEM_SAMPLES;
    if (memCount < MEM_SAMPLES) memCount++;

    // Rolling average over last ERR_SAMPLES frames
    errBuf[errIdx] = error;
    errIdx = (errIdx + 1) % ERR_SAMPLES;
    if (errIdx == 0) errFull = true;
    int n = errFull ? ERR_SAMPLES : errIdx;
    int smooth = 0;
    for (int i = 0; i < n; i++) smooth += errBuf[i];
    smooth /= n;

    // Scale smoothed error to compensate for camera being 15 cm ahead of axles
    int adjErr = smooth * CAM_SCALE_NUM / CAM_SCALE_DEN;
    int absAdj = abs(adjErr);

    if (absAdj <= ZONE_CENTER) {
        driveLeft(BASE_SPEED);
        driveRight(BASE_SPEED);
        driveFor(FORWARD_MS);

    } else if (adjErr > 0) {
        // Line is left of centre → turn left
        if (absAdj <= ZONE_FAR) {
            driveLeft(INNER_SLOW); driveRight(BASE_SPEED);
            driveFor(TURN_SHORT_MS);
        } else {
            driveLeft(INNER_REVERSE); driveRight(BASE_SPEED);
            driveFor(TURN_LONG_MS);
        }

    } else {
        // Line is right of centre → turn right
        if (absAdj <= ZONE_FAR) {
            driveLeft(BASE_SPEED); driveRight(INNER_SLOW);
            driveFor(TURN_SHORT_MS);
        } else {
            driveLeft(BASE_SPEED); driveRight(INNER_REVERSE);
            driveFor(TURN_LONG_MS);
        }
    }
}
