# Partie 1

## Question 1

**Modéliser le problème comme un processus décisionnel Markovien en donnant une récompense signicative lorsqu'on atteint la case but. (Par exemple un bonus de 1000 points mais on pourra essayer d'autres valeurs).**

PDM = (S, A, T, R)

*S = (posX, posY)*						          # ensemble des cases, entre *nblignes × nbcolonnes*

*A = {R, T, Y, U, J, H, G, F}* 					# ensemble des actions

*T ((x, y), a, (x', y')) = 1 - q÷16*    # si case prévue
*T ((x, y), a, (x' ± 1, y' ±1)) = 1/16* # si case proche et non mur
*T ((x, y), a, (x' ± 1, y' ±1)) = 0*    # sinon

*R(nblignes, nbcolonnes) = 1000*
*R(deplacement) = -1*
*R(V, B, R, N) = (-10, -20, -30, -40)*
		
**Écrire les équations de Bellman pour ce problème en utilisant un facteur d'actuation gamma.** 

w = {-1 + gamma * ; -1 + }
v
b
r
n

## Question 2

**Cas déterministe. Déterminer par itération de la valeur un chemin optimal de la case initiale vers la case but et tester sur une grille de taille ...**


