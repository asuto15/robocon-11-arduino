/*
 *
 * M5Stack SD Updater
 * Project Page: https://github.com/tobozo/M5Stack-SD-Updater
 *
 * Copyright 2018 tobozo http://github.com/tobozo
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files ("M5Stack SD Updater"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#define ROLLBACK_LABEL   "Rollback" // reload app from the "other" OTA partition
#define LAUNCHER_LABEL   "Launcher" // load Launcher (typically menu.bin)
#define SKIP_LABEL       "Skip >>|" // resume normal operations (=no action taken)
#define SAVE_LABEL       "Save"     // copy sketch binary to FS
#define BTN_HINT_MSG     "SD-Updater Lobby"
#define SDU_LOAD_TPL     "Will Load menu binary : %s\n"
#define SDU_ROLLBACK_MSG "Will Roll back"

#if !defined SDU_APP_PATH
  #define SDU_APP_PATH nullptr
#endif
#if !defined SDU_APP_NAME
  #define SDU_APP_NAME nullptr
#endif

// callback signatures
typedef void (*onProgressCb)( int state, int size );
typedef void (*onMessageCb)( const String& label );
typedef void (*onErrorCb)( const String& message, unsigned long delay );
typedef void (*onBeforeCb)();
typedef void (*onAfterCb)();
typedef void (*onSplashPageCb)( const char* msg );
typedef void (*onButtonDrawCb)( const char* label, uint8_t position, uint16_t outlinecolor, uint16_t fillcolor, uint16_t textcolor );
typedef int  (*onWaitForActionCb)( char* labelLoad, char* labelSkip, char* labelSave, unsigned long waitdelay );
typedef void (*onConfigLoad)();

// SDUpdater config callbacks and params
struct config_sdu_t
{
  fs::FS *fs = nullptr;
  int TFCardCsPin = -1;
  bool load_defaults = true;
  bool use_rollback = true;
  const char* labelMenu     = LAUNCHER_LABEL;
  const char* labelSkip     = SKIP_LABEL;
  const char* labelRollback = ROLLBACK_LABEL;
  const char* labelSave     = SAVE_LABEL;
  const char* binFileName   = SDU_APP_PATH;
  const char* appName       = SDU_APP_NAME;

  onProgressCb      onProgress       = nullptr;
  onMessageCb       onMessage        = nullptr;
  onErrorCb         onError          = nullptr;
  onBeforeCb        onBefore         = nullptr;
  onAfterCb         onAfter          = nullptr;
  onSplashPageCb    onSplashPage     = nullptr;
  onButtonDrawCb    onButtonDraw     = nullptr;
  onWaitForActionCb onWaitForAction  = nullptr;

  void setCSPin( const int param )                { TFCardCsPin = param; }
  void setFS( fs::FS *param )                     { fs = param; }
  void setProgressCb( onProgressCb cb )           { onProgress = cb; }
  void setMessageCb( onMessageCb cb )             { onMessage = cb; }
  void setErrorCb( onErrorCb cb )                 { onError = cb; }
  void setBeforeCb( onBeforeCb cb )               { onBefore = cb; }
  void setAfterCb( onAfterCb cb )                 { onAfter = cb; }
  void setSplashPageCb( onSplashPageCb cb )       { onSplashPage = cb; }
  void setButtonDrawCb( onButtonDrawCb cb )       { onButtonDraw = cb; }
  void setWaitForActionCb( onWaitForActionCb cb ) { onWaitForAction = cb; }

  void setLabelMenu( const char* label )          { labelMenu = label; }
  void setLabelSkip( const char* label )          { labelSkip = label; }
  void setLabelRollback( const char* label )      { labelRollback = label; }
  void setLabelSave( const char* label )          { labelSave = label; }
  void setAppName( const char* name )             { appName = name; }
  void setBinFileName( const char* name )         { binFileName = name; }
  void useRolllback( bool use )                   { use_rollback = use; }

};

// override this from sketch
static onConfigLoad SDUCfgLoader = nullptr;
static config_sdu_t SDUCfg;

// required by SD-Update methods
#include <esp_partition.h>
extern "C" {
  #include "esp_ota_ops.h"
  #include "esp_image_format.h"
}
// required to guess the reset reason
#if defined ESP_IDF_VERSION_MAJOR && ESP_IDF_VERSION_MAJOR >= 4
  #if defined CONFIG_IDF_TARGET_ESP32
    #include <esp32/rom/rtc.h>
  #elif defined CONFIG_IDF_TARGET_ESP32S2
    #include <esp32s2/rom/rtc.h>
  #elif defined CONFIG_IDF_TARGET_ESP32C3
    #include <esp32c3/rom/rtc.h>
  #else
    #error Target CONFIG_IDF_TARGET is not supported
  #endif
#else
  #include <rom/rtc.h>
#endif

#define resetReason (int)rtc_get_reset_reason(0)

// required to store the MENU_BIN hash
#include <Preferences.h>

#ifndef MENU_BIN
  #define MENU_BIN "/menu.bin"
#endif

#if !defined(TFCARD_CS_PIN) // override this from your sketch if the guess is wrong
  #if defined( ARDUINO_LOLIN_D32_PRO ) || defined( ARDUINO_M5STACK_Core2  ) || defined( ARDUINO_M5Stack_Core_ESP32 ) || defined( ARDUINO_M5STACK_FIRE)
    #define TFCARD_CS_PIN  4
  #elif defined( ARDUINO_ESP32_WROVER_KIT ) || defined( ARDUINO_ODROID_ESP32 )
    #define TFCARD_CS_PIN 22
  #elif defined ARDUINO_TWATCH_BASE || defined ARDUINO_TWATCH_2020_V1 || defined ARDUINO_TWATCH_2020_V2 || defined(ARDUINO_TTGO_T1)
    #define TFCARD_CS_PIN 13
  #else
    #define TFCARD_CS_PIN SS
  #endif
#endif

#if !defined SDU_HEADLESS && (defined _CHIMERA_CORE_ || defined _M5STICKC_H_ || defined _M5STACK_H_ || defined _M5Core2_H_ || defined LGFX_ONLY || defined __M5UNIFIED_HPP__)
  #define USE_DISPLAY
  #if defined _M5Core2_H_
    //#define SDU_HAS_TOUCH
  #endif
#else
  // #warning SD-Updater will run in Headless mode
#endif


