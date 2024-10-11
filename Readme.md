## :gear: Installing ROS 2 Foxy on Jetson (:us: - version) 



## :gear: Установка ROS2 Foxy на Jetson (:ru: - version) 

Установка всех необходимых компонентов происходила с использованием [Jetson NX](https://www.nvidia.com/ru-ru/autonomous-machines/embedded-systems/jetson-xavier-nx/) под управленим Jetpack 5.1.4 [L4T 35.6.0]

На Jetson NX была установлена дополнительная память NVME. Рекомендуется перенести систему на NVME накопитель, для этого необходимо выполнить следующие команды:

```bash
sudo apt install -y git
git clone https://github.com/Daniel-Robotic/rootOnNVMe.git

cd rootOnNVMe
./copy-rootfs-ssd.sh
./setup-service.sh
```

После переноса перезагружаем Jetson и производим установку дополнительных пакетов необходимых для развертывания ROS2 Foxy в Docker контейнере:

```bash
sudo apt update
sudo apt install nvidia-jetpack

echo export PATH=/usr/local/cuda/bin:$PATH >> ~/.bashrc
echo export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH >> ~/.bashrc

source ~/.bashrc 

sudo apt install -y python3-pip python3-dev
python3 -m pip install --upgrade pip

sudo -H pip3 install jetson-stats
```

После ввода команды `jtop` должна выводиться подробная статистика работы Jetson.

Следующим шагом служит клонирование репозитория с github с последующей разверткой ROS2 Foxy на jetson. Для этого выполняем следующие команды:

```bash
git clone https://github.com/Daniel-Robotic/jetson-ros-detection.git
cd jetson-ros-detection

sudo docker compose -f docker-compose-ros.yaml up --build
```

После этого начнется сборка Docker образа с последующим развертыванием ROS2 Foxy и автоматическим запуском `ros2 launch` для работы с Jetson Nx с камерой Intel Realsense D400/L500.

---
### Замечание

Используйте флаг `-d` при запуске docker compose для осоединения от docker контейнера.



