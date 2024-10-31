# EmbeddedSystemDesign_Course
Labs during theembedded system design course

# Normal class

## Lab1

UART/LCD

## Lab2

*二退*

# Laboratory class

## Lab1

UART/LCD

1. 開機後USR LED OFF
2. 按下USR BUTTON後，USR LED以1Hz速度閃爍，同時COM Port及LCD螢幕送出“LED FLASH = 1\r\n”字串。
3. 再按下USR BUTTON後，USR LED OFF並停止閃爍，同時COM Port及LCD螢幕送出“LED FLASH = 0\r\n”字串。
4. 按USR BUTTON交互控制USR LED閃爍與否。
5. COM Port參數:9600 bps,8 Bit data ,None parity, 1 stop Bit

## Lab2

數位麥克風-FFT

1. 讀取數位麥克風訊號，將訊號顯示於螢幕水平軸:0~130之間。
2. 數位麥克風取樣頻率設定8kHz 。
3. 將數位麥克風訊號做FFT後顯示於螢幕水平軸: 140~270之間。
4. 頻率顯示範圍為0~4kHz。
5. 當手點麥克風訊號軸螢幕時，可開始或停止讀取麥克風訊號。
6. 當手點FFT頻率軸螢幕時，右上角顯示該頻率數值跟功率大小

## Lab3

AI手寫辨識

1. 初始畫面顯示左右兩個框 ，左邊為手寫板 ，右邊為按鈕和結果
2. 未在手寫板寫東西時 ，點擊右邊按鈕不會有反應
3. 手寫板部分顯示觸控痕跡
4. 在手寫板寫入後， 點擊右邊框內 ，進行AI判斷並將結果顯示於右邊框內 ，顯示字形為Font57
5. 當再次按下右邊按鈕，清除螢幕並 回到初始畫面

## Lab4

簡易繪圖板

1. 開機初始時清除手寫區域
2. 按下”Open”按鈕，開啟之前”Save”的畫面
3. 按下”Save”按鈕，儲存目前畫面至SDRAM中
4. 按下”Clear”按鈕，清除手寫區域
5. 按下顏色按鈕後，手寫顏色變更為該顏色，開機後預設顏色為黑色
