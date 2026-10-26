// =============================================================
// pid_rotation.cpp
// Régulateur PID pour faire tourner le Turtlebot3 vers un angle cible
// Compatible Gazebo Harmonic + ROS2 Jazzy (TwistStamped)
// Projet Fil Rouge - Badre Debbioui - Polytech Dijon
// =============================================================

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class PIDRotation : public rclcpp::Node
{
public:
    PIDRotation() : Node("pid_rotation")
    {
        // ===== PARAMÈTRES PID (à modifier pour tes tests) =====
        Kp_ = 1.0;   // Gain proportionnel
        Ki_ = 0.0;   // Gain intégral
        Kd_ = 0.0;   // Gain dérivé

        // ===== ANGLE CIBLE en radians (90° = pi/2) =====
        angle_cible_ = M_PI / 2.0;

        // Initialisation des variables
        erreur_precedente_ = 0.0;
        erreur_integrale_ = 0.0;
        angle_actuel_ = 0.0;

        // Publisher : envoie les commandes de vitesse au robot (TwistStamped pour Gazebo Harmonic)
        publisher_cmd_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10);

        // Subscriber : lit la position du robot via l'odométrie
        subscription_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PIDRotation::odomCallback, this, std::placeholders::_1));

        // Timer : exécute le PID à 20 Hz (toutes les 50 ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&PIDRotation::pidLoop, this));

        RCLCPP_INFO(this->get_logger(), "Noeud PID Rotation démarré");
        RCLCPP_INFO(this->get_logger(), "Kp=%.2f Ki=%.2f Kd=%.2f", Kp_, Ki_, Kd_);
        RCLCPP_INFO(this->get_logger(), "Angle cible : %.2f rad (%.1f°)",
                    angle_cible_, angle_cible_ * 180.0 / M_PI);
    }

private:
    // Callback : appelé à chaque message d'odométrie reçu
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Conversion quaternion -> angle yaw (rotation autour de Z)
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        // Formule de conversion quaternion -> yaw
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        angle_actuel_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // Boucle PID : appelée toutes les 50 ms
    void pidLoop()
    {
        // 1. Calcul de l'erreur
        double erreur = angle_cible_ - angle_actuel_;

        // Normaliser l'erreur entre -pi et pi (chemin le plus court)
        while (erreur > M_PI) erreur -= 2.0 * M_PI;
        while (erreur < -M_PI) erreur += 2.0 * M_PI;

        // 2. Terme intégral (somme des erreurs)
        erreur_integrale_ += erreur * 0.05;  // dt = 50 ms

        // 3. Terme dérivé (variation de l'erreur)
        double erreur_derivee = (erreur - erreur_precedente_) / 0.05;

        // 4. Loi de commande PID
        double commande = Kp_ * erreur + Ki_ * erreur_integrale_ + Kd_ * erreur_derivee;

        // 5. Saturation : limite la vitesse angulaire (sécurité)
        if (commande > 2.84) commande = 2.84;     // max Burger
        if (commande < -2.84) commande = -2.84;

        // 6. Publication sur /cmd_vel (TwistStamped pour Gazebo Harmonic)
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.twist.linear.x = 0.0;          // pas d'avance
        msg.twist.angular.z = commande;    // rotation
        publisher_cmd_->publish(msg);

        // 7. Affichage debug
        RCLCPP_INFO(this->get_logger(),
                    "Angle: %.3f rad | Erreur: %.3f | Commande: %.3f",
                    angle_actuel_, erreur, commande);

        // 8. Mémoriser l'erreur pour le calcul dérivé
        erreur_precedente_ = erreur;
    }

    // Variables membres
    double Kp_, Ki_, Kd_;
    double angle_cible_;
    double angle_actuel_;
    double erreur_precedente_;
    double erreur_integrale_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odom_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDRotation>());
    rclcpp::shutdown();
    return 0;
}