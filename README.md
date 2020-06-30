# Extended-Kalman-Filter
## Extended Kalman Filter  
AtushiさんのMATLABcodeを参考にC++にてEKFを実装しました.
https://myenigma.hatenablog.com/entry/20130413/1365826157  
C++で演算した結果をtxtファイルに保存して, 描画はMATLABで行っています.
![EKF](https://github.com/Ramune6110/Extended-Kalman-Filter/blob/master/Extended_Kalman_Filter.png)
## Environment
Ubuntu18.04
## Procedure
```bash
g++ main.cpp extended_kalman_filter.cpp -I /usr/include/eigen3
```
```bash
./a.out
```


