## Homework 3

- 实现与作业2类似的插值算法，实现法向量、颜色、纹理颜色的插值;
- 实现 Blinn-Phong 模型;
- 实现 Texture Shading Fragment Shader;
- 实现 Bump mapping;
- 实现 displacement mapping;
- 默认在其中实现了作业2中的SSAA。

## How to Run

```bash
mkdir build
cd build
cmake ..
make -j4
./Rasterizer output.png <model>
```

## Args

```
./Rasterizer texture.png texture
```

![](images/texture.png)

```
./Rasterizer normal.png normal
```

![](images/normal.png)

```
./Rasterizer phong.png phong
```

![](images/phong.png)


```
./Rasterizer bump.png bump
```

![](images/bump.png)

```
./Rasterizer displacement.png displacement
```

![](images/displacement.png)

## Bilinear Color

使用256分辨率的贴图进行对比.

origin:

![](images/texture-256.png)

bilinear:

![](images/texture-256-bil.png)
