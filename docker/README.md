# lightning-lm Docker

这是为 lightning-lm 项目准备的 Docker 指南，分为构建和运行两部分

## 构建

可以在本地构建或者在云端构建然后拉取，如果需要在本地构建，则在安装好docker环境之后执行

```bash
cd docker
docker build -t lighting_lm_image .
```

经过若干分钟的构建（这个过程会持续半个小时以上，甚至有失败的可能），就可以完成在本地的镜像构建，当然您也可以自行指定构建出的镜像的名称

## 云端镜像拉取

为了方便大家快速使用，本人基于腾讯云完成了镜像的构建和托管，并且向大家开放下载，因为镜像源为国内源，所以可以以非常快的速度拉取，甚至可以实现两分钟内拉取并成功运行，这里给出拉取指令

```bash
docker pull docker.cnb.cool/gpf2025/slam:demo
```

同时给出存放镜像的[腾讯云网站](https://cnb.cool/gpf2025/slam/-/packages/docker/slam)，便于大家查看版本号变化

## 容器运行

当有了镜像之后就可以运行，这里建议大家将下载好的数据集挂载到容器上，这样容器中就可以播放对应的数据，并且不需要进行复制操作，这里给出运行指令及指令的解释

```bash
docker run -it --rm \
    -e "DISPLAY=$DISPLAY" \ #可视化用
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \ #可视化用
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \ #可视化用
    -v "$(pwd)/nclt.db3:/tmp/nclt.db3" \ #将本地数据包挂载到镜像中使用，根据具体情况来
    --gpus all \ #使用主机GPU
    -e "QT_X11_NO_MITSHM=1"  \#可视化用
    --network=host  \
    docker.cnb.cool/gpf2025/slam:demo #要运行的镜像名
```
