# Robotics

## TODO (28/03/2020)

- [x] Faire la todo list (sert d'exemple pour voir comment cocher)

## Notes sur l'environnement

L'environnement est découpé en carrés de largeur L. Plus la longueur L est proche de celle du robot et plus la méthode sera précise. Au contraire, un grand L rendra l'exploration plus rapide. Le robot ne peut se déplacer que du centre d'un carré vers le centre d'un autre carré. 

L'ensemble des décisions prises par le robot se font par rapport au carré sur lequel il se trouve, les 8 carrés connexes à celui du centre et un carré supplémentaire face au robot. A chaque instant, le robot n'a donc besoin de connaître l'état que de ces 10 carrés (obstacle / pas obstacle).

![alt text](https://cdn.discordapp.com/attachments/512671211998937088/693901539290513454/dir.png "zigzag_carrés")
La flèche représente l'orientation du robot.

## Machines d'états

### Zigzag

![alt text](https://cdn.discordapp.com/attachments/512671211998937088/693910118802260039/Untitled_Diagram.png "zigzag")

1. Front capture

   Le robot utilise son capteur de distance pour prendre une capture face à lui. Le but est de capturer le nombre de
   collisions dans le carré devant lui, celui devant à gauche et celui devant à droite. A partir d'un threshold, on décide  
   s'il y a ou non un obstacle dans ces trois carrés (le threshold permet d'éviter le bruit). L'ensemble des points capturés 
   étant des collisions sont ajoutés dans une liste de points.

   Un carré supplémentaire en face du robot est capturé, il servira à la navigation.
   
   L'ensemble des carrés est ajouté à la structure des carrés s'ils n'y sont pas déjà.

2. Trajectory planning

   Si le robot est capable de récupérer les 10 carrés dont il a besoin, il décide d'une nouvelle trajectoire et analyse 
   les différents carrés. Sinon il évalue juste une trajectoire en rotation de 90°

3. Analyze
   A partir des 8 carrés connexes et de s'ils sont des obstacles ou non on va décider de si oui on non le carré de gauche 
   (resp. de droite) devient un checkpoint fort, un checkpoint faible ou rien. Cela servira dans l'autre machine d'état. 
   Le niveau du checkpoint est lié au carré et on l'ajoute également dans une liste.

      ![alt text](https://cdn.discordapp.com/attachments/512671211998937088/693898109591486495/Capture_decran_du_2020-03-29_21-01-14.png "zigzag_decision")

4. Move

   Le robot bouge jusqu'au prochain centre en utilisant la trajectoire qu'il a reçu de l'état "control".

