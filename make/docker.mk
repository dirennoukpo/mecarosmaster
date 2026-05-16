##
## docker.mk for mecarosmaster [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecarosmaster/make
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Sat May 16 07:53:38 2026 dirennoukpo
## Last update Sun May 16 15:57:10 2026 dirennoukpo
##

# docker.mk - Makefile pour gérer ton environnement Docker/ROS2

DOCKER_COMPOSE = docker compose --env-file env/.env -f compose/docker-compose.yml
SERVICE = mecarosmaster

# Construire l'image
build:
	$(DOCKER_COMPOSE) build $(SERVICE)

# Lancer un shell interactif dans le container
bash:
	$(DOCKER_COMPOSE) run --rm $(SERVICE) bash

# Démarrer le container en arrière-plan
up:
	$(DOCKER_COMPOSE) up -d $(SERVICE)

# Arrêter le container
down:
	$(DOCKER_COMPOSE) down

# Recompiler le workspace dans le container
colcon-build:
	$(DOCKER_COMPOSE) run --rm $(SERVICE) bash -c "cd /mecarosmaster_ws && colcon build"

# Lancer un node ROS2 (exemple)
run-node:
	$(DOCKER_COMPOSE) run --rm $(SERVICE) bash -c "source /mecarosmaster_ws/install/setup.bash && ros2 run mon_package mon_node"

# Lancer un launch file ROS2 (exemple)
launch:
	$(DOCKER_COMPOSE) run --rm $(SERVICE) bash -c "source /mecarosmaster_ws/install/setup.bash && ros2 launch mon_package my_launch_file.launch.py"
