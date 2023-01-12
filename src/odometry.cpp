#include "odometry.h"


void Odometry::update(){

    // récup val chaque encoder
    int delta_encoder_1 = e1->get_value();
    int delta_encoder_2 = e2->get_value();
    int delta_encoder_3 = e3->get_value();
    //déduire la vitesse en divisant la valeur par la différence de temp millis (arduino .h) - lastMillis
    uint32_t temp_millis = millis();
    float delta_time = static_cast<float>(temp_millis - lastMillis)/1000.0;
    // mettre a jour lastmillis
    lastMillis = temp_millis;
    //mettre set val v1,v2,v3
    v1 = delta_encoder_1*INCREMENT_TO_METRE/delta_time;
    v2 = delta_encoder_2*INCREMENT_TO_METRE/delta_time;
    v3 = delta_encoder_3*INCREMENT_TO_METRE/delta_time;
    //mutiplier par la matrix motors_to_axis
    Eigen::Vector3d vmotors(v1, v2, v3);
    Eigen::Vector3d v_robot = this->motors_to_axis * vmotors;
    //set ensuite vx_robot, vy_robot, vtheta
    vx_robot = v_robot(0);
    vy_robot = v_robot(1);
    vtheta = v_robot(2)/RAYON;
    
    //mise à jour theta

    theta += vtheta * delta_time;

    //matrice de rotation et calcul des x, y dans le repere table (TODO)


}
