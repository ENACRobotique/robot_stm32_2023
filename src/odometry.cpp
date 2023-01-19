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
    Eigen::Vector3d deltas(delta_1, delta_2, delta_3);
    Eigen::Vector3d deltas_local_robot = motors_to_axis * deltas;
    
    //mise à jour deltas_repere_locals
    float delta_xr = deltas_local_robot(0);
    float delta_yr = deltas_local_robot(1);
    float delta_theta = deltas_local_robot(2)/RAYON;

    theta += delta_theta;

    vx_robot = delta_xr/delta_time;
    vy_robot = delta_yr/delta_time;

    float cos_t = cos(theta);
    float sin_t = sin(theta);
    
    float delta_x = cos_t * delta_xr - sin_t * delta_yr;
    float delta_y = sin_t * delta_xr + cos_t * delta_yr;

    vx = delta_x/delta_time;
    vy = delta_y/delta_time;

    x += delta_x;
    y += delta_y;
}
