# uv-irradiator

紫外線照射装置

----

## 自分用メモ

### コントローラのコネクタピン配置

|RP2040|上面|底面|RP2040|
|:--:|:--:|:--:|:--:|
||GND|3V3||
||n/a|ATEMP|GP28 (ADC2)|
|GP4|DIM0|DIM1|GP5|
|GP6|DIM2|DIM3|GP7|
|GP8|DIM4|DIM5|GP8|
||GND|VUVLED||
||GND|VUVLED||
||GND|VUVLED||
||n/a|FAN+||
||n/a|FAN-||
