#include "odometry.h"
#include "math.h"

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
    float delta_1 = delta_encoder_1*INCREMENT_TO_METRE;
    float delta_2 = delta_encoder_2*INCREMENT_TO_METRE;
    float delta_3 = delta_encoder_3*INCREMENT_TO_METRE;

    v1 = delta_1/delta_time;
    v2 = delta_2/delta_time;
    v3 = delta_3/delta_time;

    //mutiplier par la matrix motors_to_axis
    Eigen::Vector3d vmotors(v1, v2, v3);
    Eigen::Vector3d deltas(delta_1, delta_2, delta_3);
    Eigen::Vector3d v_robot = motors_to_axis * vmotors;
    Eigen::Vector3d deltas_local_robot = motors_to_axis * deltas;
    //set ensuite vx_robot, vy_robot, vtheta
    vx_robot = v_robot(0);
    vy_robot = v_robot(1);
    vtheta = v_robot(2)/RAYON;
    
    //mise à jour deltas_repere_locals
    float delta_x_robot = deltas_local_robot(0);
    float delta_y_robot = deltas_local_robot(1);
    float delta_theta = deltas_local_robot(2)/RAYON;

    theta += delta_theta;

    //matrice de rotation et calcul des x, y dans le repere table (TODO)
    float cos_t = cos(theta);
    float sin_t = sin(theta);

    float deltax_global = cos_t * delta_x_robot - sin_t * delta_y_robot;
    float deltay_global = sin_t * delta_x_robot + cos_t * delta_y_robot;

    x += deltax_global;
    y += deltay_global;

}
