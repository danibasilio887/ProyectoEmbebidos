#include <Arduino.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "esp32-hal-ledc.h"
#include "sdkconfig.h"
#include "camera_pins.h"

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// ================= CONFIGURACIÓN LED =================
// Usamos el canal 4 para el LED (La cámara suele usar el 0 y 1)
#define LED_LEDC_CHANNEL 4 
#define LED_LEDC_FREQ    5000
#define LED_LEDC_RES     8

// Variables globales
bool isStreaming = false;
httpd_handle_t stream_httpd = NULL;

// Estructura para stream multipart
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

// ==================== CONTROL DEL FLASH ====================
void setupLedFlash() {
#if defined(LED_GPIO_NUM)
    // Configuración compatible con ESP32 Core v2.x (Estándar en PlatformIO)
    pinMode(LED_GPIO_NUM, OUTPUT);
    ledcSetup(LED_LEDC_CHANNEL, LED_LEDC_FREQ, LED_LEDC_RES);
    ledcAttachPin(LED_GPIO_NUM, LED_LEDC_CHANNEL);
    ledcWrite(LED_LEDC_CHANNEL, 0); // Empezar apagado
#endif
}

void setLedIntensity(int intensity) {
#if defined(LED_GPIO_NUM)
    intensity = constrain(intensity, 0, 255);
    // En v2.x escribimos al CANAL, no al PIN
    ledcWrite(LED_LEDC_CHANNEL, intensity);
#endif
}

// ==================== HANDLER DEL STREAM ====================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  // CORS Headers vitales para que el Host pueda mostrar la imagen
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "30");

  isStreaming = true;
  
  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Fallo captura de camara");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          Serial.println("Fallo compresion JPEG");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
      if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    
    if (res != ESP_OK) {
      break;
    }
  }

  isStreaming = false;
  return res;
}

// ==================== HANDLER DE CONTROL ====================
static esp_err_t cmd_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else { free(buf); httpd_resp_send_404(req); return ESP_FAIL; }
        } else { free(buf); httpd_resp_send_404(req); return ESP_FAIL; }
        free(buf);
    } else { httpd_resp_send_404(req); return ESP_FAIL; }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    if (!strcmp(variable, "flash")) {
        setLedIntensity(val);
    }
    else if (!strcmp(variable, "quality")) {
        res = s->set_quality(s, val);
    }
    else if (!strcmp(variable, "framesize")) {
        if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
    }
    else {
        res = -1;
    }

    if (res) return httpd_resp_send_500(req);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81; 
  
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };
  
  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    httpd_register_uri_handler(stream_httpd, &cmd_uri);
  }
  
  setupLedFlash(); // Inicializar el LED después del servidor
}