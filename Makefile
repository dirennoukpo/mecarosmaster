##
## Makefile for mecarosmaster [SSH: ROSMASTER-YAHBOOM] in /home/rosmaster/mecarosmaster
##
## Made by dirennoukpo
## Login   <diren.noukpo@epitech.eu>
##
## Started on  Sat May 16 07:53:42 2026 dirennoukpo
## Last update Sun May 16 13:24:40 2026 dirennoukpo
##

# Makefile principal

include make/docker.mk

# Alias pratiques
docker-build: build
docker-up: up
docker-down: down
docker-bash: bash
docker-colcon: colcon-build
docker-run-node: run-node
docker-launch: launch

# Cible par défaut
.DEFAULT_GOAL := docker-bash