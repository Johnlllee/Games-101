## Homework 8

- 绳子约束，正确的构造绳子;
- 半隐式欧拉法;
- 显式欧拉法;
- 显式Verlet;

## How to Run

```bash
mkdir build
cd build
cmake ..
make -j4
./ropesim -s 1024
```

## Results

- `stepbyframe=1024`

![](images/1024.gif)

- `stepbyframe=2048`

![](images/2048.gif)

- `stepbyframe=4096`

![](images/4096.gif)