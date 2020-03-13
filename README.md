# Robotics

## Machine d’état

### Find obstacle :
Avancer en ligne droite jusqu’à détecter un objet à portée (peu importe la direction).	

### Is the obstacle known ?
Vérifie que l’obstacle n’a pas encore été cartographié en regardant à proximité s’il n’y a 
pas des points déjà ok (à proximité car on n’est pas sûr que le même point physique sera 
deux fois exactement au même endroit pour le capteur).

### Avoid obstacle :
S’orienter vers
1. l’obstacle le plus proche non cartographié déjà observé
2. si aucun obstacle disponible, l’endroit le moins observé de la map

### Avoid collision :
Approcher l’obstacle en s’allignant avec (prêt à le suivre, l’obstacle doit être à droite).

### Follow obstacle :
Enregistrer le point de départ. On reste à distance constante de l’obstacle (attention à gérer les
virages quand on arrive au « boût »). S’arrête quand on retombe au point de départ de cet état.

### Process obstacle :
1. Classifier en « contour map » ou « obstacle »
2. Détecter les cercles et les extraires (si on a un cercle le long d’un carré, créer deux obstacles
: un rond et un carré)

### Is map filled ?
Vérifie s’il y a encore des lieux non explorés.

### Postprocessing :
Si besoin…

### Back to spawn :
Retour à la position initiale (ou autre position, à voir plus tard. Ne fait pas partie de l’exploration 
en tant que tel).

## Mémoire

### Carte de points (matrice) :
Reprend l’ensemble des points détectés comme étant des obstacles. Les points ont la valeur 0 quand ils 
sont vides, 1 quand ils sont un obstacle observé, 2 quand ils sont un obstacle validé (qui a été contourné),
3 quand c’est un mur externe validé (qui a été contournée).
