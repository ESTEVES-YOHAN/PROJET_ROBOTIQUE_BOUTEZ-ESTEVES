# Projet Robotique SeaTech 2A SN

Ce Projet s'inscrit dans l'Unité d'enseignement Python Objet et Simulation Robotique dispensé par M. Dinar.
Le But du projet était de nous faire manipuler ROS2 pour la programmation robotique dans un environnement de simulation tel que Gazebo.
Pour ce faire nous sommes partis d'un projet pré-existant d'un Quadcopter (https://github.com/WallNutss/EE234899-Quadcopter-Gazebo-VisualServoing?tab=readme-ov-file)

## Fonctionnalités
L'objectif a été de prendre le quadcopter fonctionnel du projet et d'ajouter une fonctionnalité.

### Quadcopter d'origine
Le Quadcopter d'origine est un appareil ayant déjà à sa disposition un contrôle sur le "roll", "pitch" et "yaw" lors de la mise en marche et des déplacements. Ce dernier possède également 2 appareils en plus des moteurs.
- Une Caméra avant : Permettant la retransmission de l'environnement du drône.
- Un Sonar : Pointant vers le sol et permettant ainsi d'obtenir avec précision la hauteur du drône.

### Fonctionnalités implémentées.
Pour notre projet nous avons voulu principalement nous orienter sur le déplacement du drône et sur l'utilisation du sonar. Nous avons donc travaillé étape par étape pour ajouter des fonctionnalités pour contrôler le drône.

- **Trajectoire Géométrique** : Circulaire et Carré (Node : circle_traj & square_traj)
- **Trajectoire Séquentielle** : On spécifie des actions dans une séquence pour que le drône réalise la séquence. (Node : seq_traj)
- **Trajectoire par spécification de location** : Ici nous allons spécifier une suite de position que le drône devra atteindre. (Node : waypoint_follower2)

De plus, malgré l'impossibilité à notre niveau d'abstraction de réellement couper les moteurs dans Gazebo, nous avons voulu utilisé le Sonar à disposition sur notre drône et nous avons ajouter à la fin de notre waypoint_follower3 un appel simulant une montée puis une chute qui est ensuite rattraper par la réactivation des moteurs en sens opposés lorsque le sonar détecte le sol à une distance inférieur ou égale à 3 mètres.

## Lancement du projet

Afin de lancer ce projet il est necessaire de réaliser une installation de ROS2 via les instructions officielles (https://docs.ros.org/en/humble/Installation.html) mais également de Gazebo (https://classic.gazebosim.org/tutorials?tut=install_ubuntu).

Ensuite il faut cloner le projet sur la machine locale :

```
git clone https://github.com/ESTEVES-YOHAN/PROJET_ROBOTIQUE_BOUTEZ-ESTEVES.git
```
Puis se placer dans le dossier du projet avant de faire :

```
colcon build
source install/setup.bash
```
Ensuite, pour lancer Gazebo ainsi que le drône :

```
ros2 launch  sjtu_drone_bringup sjtu_drone_bringup.launch.py
```

Puis pour lancer la caméra et faire décoller le drône :

```
ros2 run image_subscriber pose_display
```

### Lancement des fonctionnalités

- Atterrir :
```
ros2 topic pub /drone/land std_msgs/msg/Empty {} --once
```

- Décoller :
```
ros2 topic pub /drone/takeoff std_msgs/msg/Empty {} --once
```

- Trajectoire Circulaire :
```
ros2 run drone_trajectory circle_traj
```
- Trajectoire Carré :
```
ros2 run drone_trajectory square_traj
```
- Trajectoire séquentielle :
```
ros2 run drone_trajectory seq_traj
```
- Trajectoire Waypoint Follow :
```
ros2 run drone_trajectory waypoint_follower2
```
- Trajectoire Waypoint Follow + séquence d'urgence :
```
ros2 run drone_trajectory waypoint_follower3
```

De plus, il est possible de voir en direct les données envoyées par le sonar via le node :
```
ros2 run drone_trajectory sonar_graph
```
