#include <M5Stack.h>
//reference = https://github.com/m5stack/m5-docs/blob/master/docs/ja/api/lcd.md

//setup()関数はM5Stackがスタートした最初に読み込まれる
void setup(){
  // M5Stack オブジェクトの初期化
  M5.begin();

  //Power chipがgpio21, gpio22, I2Cにつながれたデバイスに接続される。
  //バッテリー動作の場合はこの関数を読んでください（バッテリーの電圧を調べるらしい）
  M5.Power.begin();

  M5.Lcd.setBrightness(200); //バックライトの明るさを0（消灯）～255（点灯）で制御
  //M5.Lcd.loadFont("filename", SD); // フォント読み込み
}


void loop() {
 M5.Lcd.fillScreen(WHITE);

  
 // 文字描画
  M5.Lcd.setCursor(10, 10); //文字表示の左上位置を設定
  M5.Lcd.setTextColor(RED); //文字色設定(背景は透明)(WHITE, BLACK, RED, GREEN, BLUE, YELLOW...)
  M5.Lcd.setTextSize(2);//文字の大きさを設定（1～7）
  M5.Lcd.print("Hey Guys! \n\n We have a gift for you!");

  M5.Lcd.setTextColor(RED, BLACK); //文字色設定と背景色設定(WHITE, BLACK, RED, GREEN, BLUE, YELLOW...)
  M5.Lcd.setCursor(10, 100); //文字表示の左上位置を設定
  M5.Lcd.print("Hey Guys! \n\n We have a gift for you!");


  delay(1000); // 1000ms待つ
  M5.Lcd.fillScreen(WHITE);

  

  // ディスプレイの大きさ取得（僕の環境では320 x 240だった）
  int display_width = M5.Lcd.width();
  int display_height = M5.Lcd.height();
  M5.Lcd.setCursor(10, 10); //文字表示の左上位置を設定
  M5.Lcd.printf("w=%d, h=%d",display_width, display_height);


  delay(1000); // 1000ms待つ
  M5.Lcd.fillScreen(WHITE);

  
  // 特定の色で全体を塗りつぶす
  uint16_t colors[] = {WHITE, RED, GREEN, BLUE, BLACK, YELLOW };
  for(int j = 0; j<6; j++){
    M5.Lcd.fillScreen(colors[j]);
    delay(300);
  }

   uint16_t colorvalue;
   colorvalue= M5.Lcd.color565(123,30,50); //RGB値（0～255）を指定することもできる
   M5.Lcd.fillScreen(colorvalue);


  delay(1000);// 1000ms待つ
  M5.Lcd.fillScreen(WHITE);

  // 図形を書く（長方形、円、三角形）
  M5.Lcd.drawRect(10, 10, 80, 50, BLUE); //枠だけ left, top, witdh, height
  M5.Lcd.fillRect(15, 15, 70, 40, RED); // 塗りつぶし left, top, witdh, height

  M5.Lcd.drawRoundRect(1000, 10, 80, 50, 10, GREEN); // 隅が丸くなった長方形(x,y,width,height,radius,col)
  
  M5.Lcd.drawCircle(250, 50, 40, BLUE); //枠だけ center-x, center-y, radius
  M5.Lcd.fillCircle(250, 50, 30, RED); //塗りつぶし center-x, center-y, radius
  
  M5.Lcd.drawTriangle(50, 130, 100, 130, 80,  200, BLUE);//枠だけ (x1, y1), (x2,y2), (x3, y3)
  M5.Lcd.fillTriangle(90, 130, 130, 150, 110, 200, RED);//塗りつぶし (x1, y1), (x2,y2), (x3, y3)

  M5.Lcd.drawPixel(300, 40, RED);//（x, y)のピクセルを指定の色で塗る
  M5.Lcd.drawPixel(300, 45, RED);//（x, y)のピクセルを指定の色で塗る
  M5.Lcd.drawPixel(300, 50, RED);//（x, y)のピクセルを指定の色で塗る
  M5.Lcd.drawPixel(300, 55, RED);//（x, y)のピクセルを指定の色で塗る
  
  M5.Lcd.drawLine(0, 0, 120, 120, BLACK);//（x1, y1) (x2, y2)
  M5.Lcd.drawFastVLine(150, 110, 100, RED);//(x,y,height,col) (x,y)からheightだけ垂直（Vertical）な線を書く
  M5.Lcd.drawFastHLine(150, 110, 150, GREEN);//(x,y,width,col) (x,y)からwidthだけ水平（Horizontal）な線を書く


  delay(1000);// 1000ms待つ
  M5.Lcd.fillScreen(WHITE);


  // 高度な描画
  for(uint8_t i=0;i<10;i++){
    M5.Lcd.progressBar(0,0,240,20,i*10);//(x,y, width, height, percent)
    delay(100);
  }
  M5.Lcd.qrcode("http://www.m5stack.com",10,10,230,7); //(URL,x,y,size, version)


  delay(1000);// 1000ms待つ
}
