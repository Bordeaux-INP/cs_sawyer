# Opinion pollster: Sawyer robot polling visitors on a whiteboard




# Manuel en français
## Instructions animateurs

#### 1. Marche et arrêt quotidiens

Mettre le capuchon pour ne pas faire sécher le feutre inutilement

### 2. Le robot ne vote plus
#### 2.1. Regarder les voyants des boutons
* Les 2 voyants clignotent lentement : tableau plein, effacer puis faire une RàZ (cf 3.1)
* Les 2 voyants clignotent rapidement : erreur de connexion : vérifier les branchements, redémarrer robot (cf 3.4)
* Un seul voyant clignote : un vote est en cours. Si le robot ne bouge pas, 


### 3. Dépannage animateurs
#### 3.1. Remise à zéro des compteurs

Tenir appuyer MEFIANCE et CONFIANCE pendant 10 secondes.
Cette opération remet à zéro les compteurs, à condition que le tableau soit effectivement plein et que l'écran affiche `ERREUR, le tableau est plein`

#### 3.2 Remise en service après collision `ERREUR, j'ai bousculé quelqu'un`

Cette opération n'est à réaliser que lorsque le robot affiche `ERREUR, j'ai bousculé quelqu'un`
Cette opération remet le robot en foncitonnement après qu'il ait détecté une collision.
Tenir appuyer MEFIANCE et CONFIANCE pendant 10 secondes.

#### 3.3 Déplacement manuel du bras

Cette opération peut être réalisée lorsque le robot s'est coincé dans une position singulière ou lorsqu'il ne semble plus tracer les votes.

Deux cas se présentent :
* Une collision a été détectée et le message `ERREUR, j'ai bousculé quelqu'un` est affiché : vous pouvez bouger le bras manuellement AVEC DOUCEUR pour le décoincer
* Aucune collision n'a été détectée : le bras peut être bougé en pressant le bouton en caoutchoux au bout du bras et en bougeant dans la position désirée.

#### 3.4 Redémarrage complet

Eteindre les deux tours se trouvant dans le meuble :
* Le contrôleur robot (la tour ressemble à un tableau électrique) : appuyer une seule fois sur le bouton de marche/arrêt et attendre que le robot se coupe
* Le contrôleur d'interaction (la tour ressemble à un ordinateur classique) : appuyer une seule fois sur le bouton de marche/arrêt puis débrancher et rebrancher après 65 secondes.

Eteindre la Rasberry Pi 

#### 3.5 Recalibration de la position du tableau

Attention : manipulation délicate.

## Maintenance technique avancée

ssh robot@cs-sawyer-controller.local
cd /home/robot/ros_ws/src/cs_sawyer
./start.bash
rostopic pub /cs/sawyer/buttons <TAB>
sudo service cs_sawyer status