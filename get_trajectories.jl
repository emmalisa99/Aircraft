"""
File for testing the scenarios given by M. Dossal
"""


#########################################################################################
## Scenario 1 : L’avion est déjà dans l’axe AB et les points A et B sont à même altitude.
#########################################################################################

function test_scenario1()
    """
    3 tests : 
	--> avion suivant x 
	--> avion suivant xy
    --> avion suivant xyz 
    """
    ### TODO 
    # doit modifier fichier initialisation (mettre sous forme de fonction) afin d'avoir le choix de position et d'orientation
    # modifier angle2quaternion pour etre plus général (regarder le package Rotations.jl)
    # modifier definition de la vitesse selon l'orientation (que la vitesse initiale suit la direction de l'avion)

    position = [0,0,0]
    test1 = Dict(nom=>"Avion orienté selon x",position=>position, orientation=>[])
    test2 = Dict(nom=>"Avion orienté selon xy",position=>position, orientation=>[])
    test3 = Dict(nom=>"Avion orienté selon xyz",position=>position, orientation=>[])
    println("Test : "*nom)
    println("Attendu : ")
    println("Obtenu : ")
end

# Remarque : scenario 1 : tester rapidement, semblait fonctionner


#########################################################################################
##Scenario 2 : L’avion en A est dirigé dans la bonne direction mais B est à une altitude différente de A.
#########################################################################################

function test_scenario2()
    """
    2 tests à realiser :
        --> B plus bas
        --> B plus haut
    """
    position = [0,0,0]
    test1 = Dict(nom=>"Vers le haut",position=>position, orientation=>[])
    test2 = Dict(nom=>"Vers le bas",position=>position, orientation=>[])
    println("Test : "*nom)
    println("Attendu : ")
    println("Obtenu : ")
end


#########################################################################################
## Scenario 3 :  B n’est pas dans la bonne direction mais à la bonne altitude (il faudra donc effectuer un virage horizontal). On pourra proposer une manœuvre de virage permettant de se repositionner de manière à avoir B devant vous.
#########################################################################################

function test_scenario3()
    """
    2 tests à realiser :
        --> B a droite
        --> B a gauche
    """
    position = [0,0,0]
    test1 = Dict(nom=>"Vers la droite",position=>position, orientation=>[])
    test2 = Dict(nom=>"Vers la gauche",position=>position, orientation=>[])
    println("Test : "*nom)
    println("Attendu : ")
    println("Obtenu : ")
end


#########################################################################################
## Scenario 4 : B n’est ni dans la bonne direction si à la bonne altitude.
#########################################################################################
function test_scenario2()
    """
    4 tests à realiser :
        --> B plus bas, à gauche
        --> B plus bas, à droite
        --> B plus haut, à gauche
        --> B plus haut, à droite
    """
    position = [0,0,0]
    test1 = Dict(nom=>"Vers le haut, à gauche",position=>position, orientation=>[])
    test2 = Dict(nom=>"Vers le bas, à gauche",position=>position, orientation=>[])
    test3 = Dict(nom=>"Vers le haut, à droite",position=>position, orientation=>[])
    test4 = Dict(nom=>"Vers le bas, à droite",position=>position, orientation=>[])
    println("Test : "*nom)
    println("Attendu : ")
    println("Obtenu : ")
end


#########################################################################################
##Scenario 5 : Et enfin, on souhaite arriver en B avec une direction et une vitesse prescrite (ou au moins dans une fourchette).
#########################################################################################

function test_scenario2()
    """
    3 tests à realiser, voire 6 :
        --> en B, veut arriver à une vitesse verticale comprise entre x et y
        --> en B, veut arriver avec une vitesse horizontale comprise entre  x et y
        --> en B, veut arriver à une vitesse verticale et horizontale comprise entre x et y
    """
    position = [0,0,0]
    test1 = Dict(nom=>"Vitesse horizontale",position=>position, orientation=>[])
    test2 = Dict(nom=>"Vitesse verticale",position=>position, orientation=>[])
    test3 = Dict(nom=>"Vitesse horizontale et verticale",position=>position, orientation=>[])
    println("Test : "*nom)
    println("Attendu : ")
    println("Obtenu : ")
end
