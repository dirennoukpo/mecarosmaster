# env/ — Fichiers d'environnement

Ce répertoire contient les variables d'environnement chargées par `docker-compose`.

## Fichiers

| Fichier                  | Rôle                                                           |
|--------------------------|----------------------------------------------------------------|
| `.env`                   | Variables **communes** robot et workstation                    |
| `.env.mecarosmaster`     | Variables spécifiques au **robot** (Pi, ports, IPs, capteurs) |
| `.env.workstation`       | Variables spécifiques à la **workstation** (SLAM, X11, Nav2)  |
| `.env.example`           | Template `.env` — copier et adapter                           |
| `.env.mecarosmaster.example` | Template robot                                            |
| `.env.workstation.example`   | Template workstation                                      |

## Priorité de surcharge

```
Dockerfile ENV (défauts image)
    ← .env          (commun, versionné)
        ← .env.mecarosmaster  ou  .env.workstation  (machine-spécifique)
            ← docker-compose environment: {}  (surcharge ponctuelle)
```

## Fichiers sensibles

`.env.mecarosmaster` et `.env.workstation` contiennent des adresses IP et
des noms de ports spécifiques à votre infrastructure. Ajoutez-les à
`.gitignore` si votre dépôt est public :

```
env/.env.mecarosmaster
env/.env.workstation
```
