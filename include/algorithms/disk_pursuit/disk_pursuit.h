class DiskPursuit {
    public:
        DiskPursuit(struct Core* core, float forward_velocity = 300, float max_velocity = 600);
        void set_signature(pros::vision_signature_s_t* signature);
        pros::vision_object_s_t get_closest_disk();
        float get_disk_distance(pros::vision_object_s_t object);
        float get_disk_direction(pros::vision_object_s_t object);
        ChassisVelocityPair step();
    private:
        struct Core* core;
        const float Kp = 1;
        const float Ki = 0;
        const float Kd = 0;
        const float max_vertical_angle = 52;
        const float max_horizontal_angle = 61;
        const float theta_initial = 60;
        float sensor_height;
        float forward_velocity;
        float max_velocity;
        
        float prev_error_direction;
        float total_error_direction;
};