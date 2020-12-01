"""Modifier la valeur de l'angle voulu (en degré) et l'axe de rotation"""


# Attention ce calcul ne prend pas en compte un 
# cas plus complexe où l'axe d'inclinaison n'est pas quadratiq

function angle2quaternion(angle_voulu,axes)
    nb_axes_concerne = sum(axes)
    xyz = sin( angle_voulu/2)
    w = sqrt(1 - xyz^2)
    angle_rotation = xyz * axes / nb_axes_concerne
    return w,angle_rotation[1],angle_rotation[2],angle_rotation[3]
end

print(angle2quaternion(10,[0,1,0]))

