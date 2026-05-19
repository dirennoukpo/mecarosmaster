# make/ — Règles Makefile

## Fichiers

| Fichier      | Rôle                                               |
|--------------|----------------------------------------------------|
| `docker.mk`  | Cibles build, up, down, logs, shell, colcon, clean |

## Cibles disponibles

```
make help                 Afficher toutes les cibles
make build-base           Image base robot
make build-robot          Image service robot (hérite base)
make build-workstation    Image workstation
make build-all            Toutes les images

make up-robot             Démarrer le robot
make up-workstation       Démarrer la workstation (+ xhost)
make up                   Démarrer tous les services

make down                 Arrêter tout
make restart-robot        Redémarrer robot
make restart-workstation  Redémarrer workstation

make logs-robot           Logs robot en direct
make logs-workstation     Logs workstation en direct
make shell-robot          bash dans le robot
make shell-workstation    bash dans la workstation

make build-ws-robot       colcon build dans le robot
make build-ws-workstation colcon build dans la workstation
make clean-ws             Supprimer build/ install/ log/

make clean                Supprimer conteneurs + images
make prune                Nettoyer les ressources Docker inutilisées
```
