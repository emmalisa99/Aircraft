Project MiniBee

# Objective

Final objective : Find the optimal flight plan of an aircraft to go from a point A to B. We try to find the path that take the less time and/or that consumes the less carburant. The optimal path has to respect some contraints. Some are physical, others impose to avoid cities. 

First objective : check the model to get simple trajectories

# Requirements

DifferentialEquations.jl
Plots.jl
LinearAlgebra.jl
StaticArrays.jl

# Architecture

Pour trouver des trajectoires 

 Solveurs : 
 - RK4 codé dans le fichier : ode_rk4.jl
 - solver Julia (par DifferentialEquations.jl : pleins de solveurs possibles normalement)

 Modèle : 
 - codé dans le fichier : model_ode.jl

Initialisation :
- initalisation : permet de dire les paramètres spécifiques à une situation de vol - state et control (durée, dt, angle de départ, poussée initial...)
- structure_and_data : définition de données non modifiable générales (gravité...) ou spécifiques (surface ailaire,aspect ratio...) 
- coefficients.jl : fichier qui permet de définir les coefficients lift et drag (pour le calcul des forces) en fonction de courbes (trouvées sur internet)

 Calcul - tracer des trajectoires : 
 - le fichier results_ode.jl fait appel au solveur et au model pour calculer la trajectoire, ce fichier fait également appel à resultat_lib.jl pour tracer les trajectoires.

Autres fichiers : 
- equations_equilibre.jl : contient des fonctions qui permettent de trouver les paramètres favorisant un équilibre (c'est à dire une annulation d'une ou plusieurs composantes de la dérivée de la vitesse)
- angle_lib.jl : fichier pour simuler des variations d'angles/pitch selon certaines fonctions (polynome, spline) 
- Fonction score
    - score_function.jl : contient une fonction pour une première mesure du score/coût de la trajectoire obtenue 


# Résultat 

Code permettant d'avoir de premières trajectoires : 
- trajectoire rectiligne et uniforme : ok
- trajectoire accélérante : pas codé
- trajectoire montante ou descendante : ok en garant même pitch et même poussée à la fin, deux angles différents et poussées : pas trouvé
- virage  : à revoir (à gauche : ok, à droite : pb car la poussée diminue en augmentant roll)
- mix : pas fait