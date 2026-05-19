# docker/scripts/

Scripts utilitaires internes aux conteneurs (non copiés en entrypoint).

Exemples d'usage futur :
- `setup_colcon_mixin.sh` — initialiser les mixins colcon (release, debug, ccache)
- `check_serial.sh` — vérifier la présence et les droits du port série
- `wait_for_ros.sh` — attendre qu'un noeud ROS2 soit actif avant de continuer
