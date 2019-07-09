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

Appuyer pendant plus de 20 secondes sur MEFIANCE et CONFIANCE, les deux boutons clignotent à vitesse rapide et la respiration s'arrềte. Tenir appuyer le bouton en caoutchouc au bout du bras pour passer en mode zéro gravité (le robot se laisse manipuler). Positionner le robot en position d'écriture sur le baton le plus en bas à gauche possible dans la zone MEFIANCE, comme s'il venait de terminer le dessin du bâton, toujours en contact avec le tableau. Appuyer à nouveau 20 secondes sur MEFIANCE et CONFIANCE jusqu'à ce qu'ils s'éteignent. Un calcul d'environ 1 à 2 minute vérifie que tous les bâtons pourront être dessinés dans cette zone. Si oui, le robot revient en position normale et il est à nouveau prêt à voter. Sinon, MEFIANCE et CONFIANCE clignotent rapidement de nouveau et il est nécessaire de définir une autre position.

# Start and manage the setup in production

See [production](./install#activate-autostart-on-this-machine).

## Some commands to help start debugging
```
ssh robot@cs-sawyer-controller.local
cd /home/robot/ros_ws/src/cs_sawyer
./start.bash
rostopic pub /cs/sawyer/buttons <TAB>   # Will list all available ROS topics
```
