#include "odometry.h"
#include "math.h"

void Odometry::update(){

    // récup val chaque encoder
    int delta_encoder_1 = e1->get_value();
    int delta_encoder_2 = e2->get_value();
    int delta_encoder_3 = e3->get_value();
    //déduire la vitesse en divisant la valeur par la différence de temp millis (arduino .h) - lastMillis
    uint32_t temp_millis = millis();
    double delta_time = static_cast<double>(temp_millis - lastMillis)/1000.0;
    // mettre a jour lastmillis
    lastMillis = temp_millis;
    //mettre set val v1,v2,v3
    double delta_1 = delta_encoder_1*INCREMENT_TO_METRE;
    double delta_2 = delta_encoder_2*INCREMENT_TO_METRE;
    double delta_3 = delta_encoder_3*INCREMENT_TO_METRE;

    v1 = delta_1/delta_time;
    v2 = delta_2/delta_time;
    v3 = delta_3/delta_time;

    //mutiplier par la matrix motors_to_axis
    Eigen::Vector3d deltas(delta_1, delta_2, delta_3);
    Eigen::Vector3d deltas_local_robot = motors_to_axis * deltas;
    
    //mise à jour deltas_repere_locals
    double delta_xr = deltas_local_robot(0);
    double delta_yr = deltas_local_robot(1);
    double delta_theta = deltas_local_robot(2)/RAYON;

    theta += delta_theta;

    vx_robot = delta_xr/delta_time;
    vy_robot = delta_yr/delta_time;

    double cos_t = cos(theta);
    double sin_t = sin(theta);
    
    double delta_x = cos_t * delta_xr - sin_t * delta_yr;
    double delta_y = sin_t * delta_xr + cos_t * delta_yr;

    vx = delta_x/delta_time;
    vy = delta_y/delta_time;

    x += delta_x;
    y += delta_y;
}


void Odometry::print_odometry() {
    Serial.print(x,3);
    Serial.print("  ");
    Serial.print(y,3);
    Serial.print("  ");
    Serial.println(theta,3);
}
