# RAPPORT [AI06] - Clément Girard, Alexandre Jörgensen

## Introduction

Lors des TPs d'AI06, nous avons pu découvrir les robots **Turtlebot**. Les éléments principaux de ces robots sont un mini ordinateur **Raspberry Pi 3**, 2 **servomoteurs Dynamixel** série X, une carte de contrôle open source programmable en Arduino et de nombreux capteurs (gyroscope, accéléromètre, boussole, sharp, caméra et **Lidar**) pour la navigation et la détection de l'environnement. Alors que nous avons pu mettre en oeuvre de nombreux capteurs dans chacun des TP, le projet final se place en conclusion et rassemble les nombreuses compétences et codes que nous avons pu utiliser.

Côté logiciel, nous avons utilisé la combinaison de **ROS** *Robot Operating System* et de **Python** sur une machine virtuelle tournant sous **Linux**. ROS est un ensemble de bibliothèques logicielles et d'outils qui aident à créer des applications robotiques. Des pilotes aux algorithmes de pointe, en passant par de puissants outils de développement. Quant à lui Python est un langage de programmation interprété de haut niveau à usage général. La philosophie de conception de Python met l'accent sur la lisibilité du code avec son utilisation notable d'une indentation importante et d'une syntaxe puissante.

### Rappel du projet

Nous avons choisi de travailler sur le projet n°4 dénommé **Caméra & LiDAR & Odométrie**. Pour rappel, le robot devra êrte capable de cartographier, suivre une cible et revenir à sa position initiale en toute sécurité. La cible, un objet de couleur unie, distincte et connue est détectée par caméra. Le robot est naturellement statique mais cherche à se rapprocher de la cible lorsqu’il en détecte une et s’oriente de manière à centrer la cible dans son image. En parallèle de cela, le LiDAR est utilisé pour détecter d’éventuels obstacles dans sa trajectoire. Lorsque trop proche d’un obstacle, le robot s’arrête pour signaler qu’il a besoin qu’on le déplace manuellement. Il garde une mémoire d’où se trouvent les obstacles à l’aide d’une grille d’occupation . Lorsqu’une cible n’est plus détectée pendant plus de cinq secondes, le robot, qui sait où il se trouve grâce à son odométrie, cherche à revenir à son point de départ en évitant les obstacles cartographiés.

### Prises de liberté sur le sujet

Connaissant les limites posés par l'utilisation de caméras dans un suivi d'objets, *variation de la luminosité empêchant un fonctionne correcte et des recalibrations intempestives* nous avons décidé de  modifier légèrement ce sujet qui nous inspirait vraiment. Nous avons donc demandé s'il était possible de remplacer la caméra par le Lidar pour le suivit d'objet. Cette modification nous ayant été accordée, nous avons pu débuter le projet en sachant ce que nous voulons atteindre comme objectifs. 

Dans ce projet, nous utiliserons donc le Lidar à la fois pour suivre l'objet et pour cartographier la zone, et les capteurs odométriques pour calculer la position du robot.
