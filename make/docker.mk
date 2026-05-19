# =============================================================================
# make/docker.mk — Cibles Docker pour mecarosmaster
# Inclus dans le Makefile principal via : include make/docker.mk
# =============================================================================

COMPOSE_FILE   := compose/docker-compose.yml
ENV_FILE       := env/.env
ROS_DISTRO     ?= humble
REGISTRY       ?= local

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------

.PHONY: build-base
build-base: ## Construire l'image base robot (Dockerfile.base)
	docker build --no-cache\
		--file service/mecarosmaster/Dockerfile.base \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) \
		--tag $(REGISTRY)/mecarosmaster-base:$(ROS_DISTRO) \
		.

.PHONY: build-robot
build-robot: build-base ## Construire l'image service robot (Dockerfile.mecarosmaster)
	docker build --no-cache\
		--file service/mecarosmaster/Dockerfile.mecarosmaster \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) \
		--build-arg REGISTRY=$(REGISTRY) \
		--tag $(REGISTRY)/mecarosmaster:$(ROS_DISTRO) \
		.

.PHONY: build-workstation
build-workstation: ## Construire l'image workstation (Dockerfile.workstation)
	docker build --no-cache\
		--file service/mecarosmaster/Dockerfile.workstation \
		--build-arg ROS_DISTRO=$(ROS_DISTRO) \
		--tag $(REGISTRY)/mecarosmaster-workstation:$(ROS_DISTRO) \
		.

.PHONY: build-all
build-all: build-base build-robot build-workstation ## Construire toutes les images

# ---------------------------------------------------------------------------
# Up / Down
# ---------------------------------------------------------------------------

.PHONY: up-robot
up-robot: ## Démarrer le service robot
	docker compose -f $(COMPOSE_FILE) --profile robot up -d mecarosmaster

.PHONY: up-workstation
up-workstation: ## Démarrer le service workstation (avec X11)
	xhost +local:docker
	docker compose -f $(COMPOSE_FILE) --profile workstation up -d workstation

.PHONY: up
up: ## Démarrer tous les services (robot + workstation)
	docker compose -f $(COMPOSE_FILE) --profile robot --profile workstation up -d

.PHONY: down
down: ## Arrêter tous les services
	docker compose -f $(COMPOSE_FILE) --profile robot --profile workstation down

.PHONY: restart-robot
restart-robot: ## Redémarrer le service robot
	docker compose -f $(COMPOSE_FILE) --profile robot restart mecarosmaster

.PHONY: restart-workstation
restart-workstation: ## Redémarrer le service workstation
	docker compose -f $(COMPOSE_FILE) --profile workstation restart workstation

# ---------------------------------------------------------------------------
# Logs & shell
# ---------------------------------------------------------------------------

.PHONY: logs-robot
logs-robot: ## Suivre les logs du robot
	docker compose -f $(COMPOSE_FILE) --profile robot logs -f mecarosmaster

.PHONY: logs-workstation
logs-workstation: ## Suivre les logs de la workstation
	docker compose -f $(COMPOSE_FILE) --profile workstation logs -f workstation

.PHONY: shell-robot
shell-robot: ## Ouvrir un shell dans le conteneur robot
	docker exec -it mecarosmaster bash

.PHONY: shell-workstation
shell-workstation: ## Ouvrir un shell dans le conteneur workstation
	docker exec -it mecarosmaster_workstation bash

# ---------------------------------------------------------------------------
# Nettoyage
# ---------------------------------------------------------------------------

.PHONY: clean
clean: down ## Supprimer les conteneurs et images du projet
	docker rmi -f \
		$(REGISTRY)/mecarosmaster-base:$(ROS_DISTRO) \
		$(REGISTRY)/mecarosmaster:$(ROS_DISTRO) \
		$(REGISTRY)/mecarosmaster-workstation:$(ROS_DISTRO) 2>/dev/null || true

.PHONY: prune
prune: ## Supprimer toutes les ressources Docker inutilisées (dangling)
	docker system prune -f

# ---------------------------------------------------------------------------
# Colcon (dans les conteneurs)
# ---------------------------------------------------------------------------

.PHONY: build-ws-robot
build-ws-robot: ## Compiler le workspace dans le conteneur robot
	docker exec -it mecarosmaster bash -c \
		"source /opt/ros/$$ROS_DISTRO/setup.bash && \
		 cd /mecarosmaster_ws && \
		 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

.PHONY: build-ws-workstation
build-ws-workstation: ## Compiler le workspace dans le conteneur workstation
	docker exec -it mecarosmaster_workstation bash -c \
		"source /opt/ros/$$ROS_DISTRO/setup.bash && \
		 cd /mecarosmaster_ws && \
		 colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"

.PHONY: clean-ws
clean-ws: ## Nettoyer le workspace (build/ install/ log/)
	docker exec -it mecarosmaster bash -c \
		"cd /mecarosmaster_ws && rm -rf build install log"
