# Turtlebot3 Robot Süpürge 

## Proje Hakkında

Bu proje, **ROS Noetic** ve **TurtleBot3** kullanılarak geliştirilmiş, **duvar takibi** ile haritalama yaparak **qr kod doğrulama sistemi** ile odaları gezmektedir.

---

## Gereksinimler

* Ubuntu 20.04
* ROS Noetic
* TurtleBot3 (Waffle)
* Gazebo 11

---

## Kurulum Aşamaları

### ROS Noetic Kurulumu

Ubuntu 20.04 üzerinde ROS Noetic kurulu olmalıdır.

### Workspace Hazırlığı

Repo'yu aşağıdaki komutları kullanarak workspace'inize ekleyin.

```bash
cd ~/catkin_ws/src
git clone https://github.com/nidakaplan/robot_supurge.git
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```

---

## Projenin Çalıştırılması


### Duvar Takibi ile Haritalamanın Yapılması

Turtlebot3 sağ duvar takibi algoritması kullanarak tüm evi dolaşarak haritalamayı gerçekleştirecektir.

```bash
roslaunch robot_supurge slam.launch
```

TÜm ev dolaşıldıktan sonra haritayı kaydedin.

```bash
$ rosrun map_server map_saver -f ~/catkin_ws/src/robot_supurge/map/map
```

## Önemli not: 
* Eğer kaydettiğiniz haritayı kullanmak istiyorsanız **catkin_ws/src/launch/localization.launch** dosyasındaki <arg name="map_file" value="$(find robot_supurge)/map/map_yedek.yaml"/> satırını <arg name="map_file" value="$(find robot_supurge)/map/map.yaml"/> ile değiştirin.

### Temizlik Rotasının Oluşturulması

**config** klasörü içerisindeki **mission.yaml** dosyasını geçerli parametrelerle gitmesini istediğiniz odalara göre doldurun.
Aşağıdaki komutu çalıştırdıktan sonra **2D Pose Estimate** ile robotun konumunu belirleyin.

```bash
roslaunch robot_supurge navigation.launch
```

Ardından yeni bir terminalde aşağıdaki komutu çalıştırarak **mission.yaml** dosyasında verdiğiniz odalara gitmesini sağlayın.

```bash
rosrun robot_supurge task_manager.py
```

### 4️⃣ Robot Süpürge Davranışı

```bash
rosrun robot_supurge robot_supurge.py
```

---

## NOT
Robot temizlik rotasını doğru bir şekilde gerçekleştirememektedir.

## 📌 Notlar

Bu proje bir **üniversite dersi final ödevi** kapsamında geliştirilmiştir.
