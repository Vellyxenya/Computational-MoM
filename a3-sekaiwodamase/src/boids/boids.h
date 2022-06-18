#ifndef BOIDS_H
#define BOIDS_H
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Sparse>
#include <Eigen/LU> //Need this for Matrix::inverse()
#include <math.h>
#include <list> //Linked list data structure for performance

template <typename T, int dim>
using Vector = Eigen::Matrix<T, dim, 1, 0, dim, 1>;

template <typename T, int n, int m>
using Matrix = Eigen::Matrix<T, n, m, 0, n, m>;

// add more for yours
enum MethodTypes {
    FREEFALL=0, SEPARATION=1, ALIGNMENT=2,
    COHESION=3, LEADER=4, ROTATION=5,
    COLLISION_AVOIDANCE=6, TEAMS=7,
    COLLAB_ADV=8
};

enum IntegrationMethods {
    SYMPLECTIC_EULER=0, EXPLICIT_EULER=1, EXPLICIT_MIDPOINT=2
};

using T = double;
using VectorXT = Matrix<T, Eigen::Dynamic, 1>;
using TV = Vector<T, 2>; //2D vector
using TM = Matrix<T, 2, 2>; //2x2 matrix

using std::cout;
using std::endl;

class Boids {
    
public:
    int dim = 2;
    double h = 0.01; //step size

    double cohesion_radius = 0.3; //neighbor search radius
    double cohesion_strength = 5;

    double alignment_radius = 0.15;
    double alignment_strength = 0.6;

    double separation_radius = 0.5;
    double separation_strength = 8;

    TV obstacle_pos = TV(0.3, 1);
    double obstacle_radius = 0.3;
    double collision_avoidance_strength = 0.01;

    TV leader_pos = TV(0.5, 0);
    double follow_leader_strength = 3;
    double leader_close_enough_radius = 0.2;

    double creation_radius = 0.07;
    double destruction_radius = 0.09;
    int cooldown_delay = 300;

    double canvas_radius = 1.2; //avoid the boids going out of screen
    bool only_1_team = false;
    
private:
    std::list<TV> positions;
    std::list<TV> velocities;
    std::list<bool> teams;
    std::list<int> cooldowns;
    std::list<TV>::iterator it_p;
    std::list<TV>::iterator it_v;
    std::list<bool>::iterator it_t;
    std::list<int>::iterator it_c;

    int n;
    bool update = true;

public:
    Boids() :n(1) {}

    Boids(int n, MethodTypes type) :n(n) {
        initializePositions(type);
    }

    ~Boids() {}

    double rand_0_1() {
        return static_cast <T> (rand()) / static_cast <T> (RAND_MAX);
    }

    TV rand_TV() {
        return TV(rand_0_1(), rand_0_1());
    }

    void initializePositions(MethodTypes type) {
        positions.clear();
        velocities.clear();
        teams.clear();
        cooldowns.clear();

        for(int i = 0; i < n; i++) {
            positions.push_back(1.25 * (rand_TV() - TV(0.5, 0.5))); //random positions
            if(type == TEAMS || type == COLLAB_ADV) {
                teams.push_back(i%2 == 0);
            } else {
                teams.push_back(true);
            }
            cooldowns.push_back(0);
        }
        switch (type) {
        case FREEFALL:
        case COHESION:
        case SEPARATION:
        case ALIGNMENT:
        case LEADER:
        case TEAMS:
        case COLLAB_ADV:
            for(int i = 0; i < n; i++) {
                velocities.push_back(0.2 * (rand_TV() - TV(0.5, 0.5))); //random velocities
            }
            break;
        case ROTATION:
            for(auto it = positions.begin(); it != positions.end(); ++it) {
                velocities.push_back(TV(-(*it)[1], (*it)[0])); //orthogonal velocities
            }
            break;
        case COLLISION_AVOIDANCE:
            for(int i = 0; i < n; i++) {
                velocities.push_back(0.5 * (rand_TV() - TV(0.5, 0))); //random velocities
            }
        default:
            break;
        }
    }

    TV getObstaclePosition() {
        return obstacle_pos;
    }

    double getObstacleRadius() {
        return obstacle_radius;
    }

    TV const_downward_force(const TV pos) {
        TV force;
        force << 0, 1;
        return force;
    }

    TV central_force(const TV pos) {
        TV force = -pos;
        return force;
    }

    TV cohesion_force(const TV pos) {
        double ref = cohesion_radius * cohesion_radius;
        TV centroid = TV::Zero();
        int nb_neighbors = 0;
        for(auto it = positions.begin(); it != positions.end(); it++) {
            if(it == it_p) continue;
            TV p2 = *it;
            double dist_squared = (pos - p2).squaredNorm();
            if(dist_squared <= ref) {
                centroid += p2;
                nb_neighbors++;
            }
        }
        if(nb_neighbors == 0) return TV::Zero();
        centroid /= nb_neighbors;
        TV f = cohesion_strength * (centroid - pos);
        return f;
    }

    TV separation_force(const TV pos) {
        double ref = separation_radius * separation_radius;
        TV repulsive_vector = TV::Zero();
        for(auto it = positions.begin(); it != positions.end(); it++) {
            if(it == it_p) continue;
            TV p2 = *it;
            double dist_squared = (pos - p2).squaredNorm();
            if(dist_squared <= ref) {
                repulsive_vector += (pos - p2).normalized() / (pos - p2).norm();
            }
        }
        TV f = 0.0005 * separation_strength * repulsive_vector;
        return f;
    }

    TV alignment_force(const TV pos) {
        double ref = alignment_radius * alignment_radius;
        TV alignment = TV::Zero();
        int nb_neighbors = 0;
        for(auto it = positions.begin(), itv = velocities.begin(); it != positions.end(); it++, itv++) {
            if(it == it_p) continue;
            TV p2 = *it;
            double dist_squared = (pos - p2).squaredNorm();
            if(dist_squared <= ref) {
                alignment += *itv;
                nb_neighbors++;
            }
        }
        if(nb_neighbors == 0) return TV::Zero();
        alignment /= nb_neighbors;
        TV f = alignment_strength * alignment;
        return f;
    }

    TV alignment_force_2(const TV pos) {
        double ref = alignment_radius * alignment_radius;
        TV alignment = TV::Zero();
        int nb_neighbors = 0;
        for(auto it = positions.begin(), itv = velocities.begin(); it != positions.end(); it++, itv++) {
            if(it == it_p) continue;
            TV p2 = *it;
            double dist_squared = (pos - p2).squaredNorm();
            if(dist_squared <= ref) {
                double dot = ((*it_v).normalized()).dot((*itv).normalized());
                if(dot > 0) { //if going in more or less the same direction
                    alignment += *itv * (1-dot); //penalize adding more alignment force for already aligned boids
                } else {
                    alignment += *itv;
                }
                nb_neighbors++;
            }
        }
        if(nb_neighbors == 0) return TV::Zero();
        alignment /= nb_neighbors;
        TV f = alignment_strength * alignment;
        return f;
    }

    TV collision_avoidance_force(const TV pos) {
        double distance = (pos - obstacle_pos).norm() - obstacle_radius;
        TV f = collision_avoidance_strength * (pos - obstacle_pos).normalized() / (distance * distance);
        double dot = std::max(0., f.normalized().dot(-(*it_v).normalized()));
        return f * dot;
    }

    TV leader_force(const TV pos) {
        TV vec = (leader_pos - pos);
        if(vec.norm() < leader_close_enough_radius) { //If close enough no force applied
            return TV::Zero();
        } else { //Too far from leader so apply force
            TV f = vec * follow_leader_strength; //Apply force proportional to distance
            double dot = vec.dot((*it_v).normalized());
            if(dot > 0) //moving towards the leader, so decelerate
                f += -(*it_v);
            return f;
        }
    }

    TV additive_force(const TV pos) {
        TV f;
        f += cohesion_force(pos);
        f += 0.5 * alignment_force(pos);
        f += 5 * separation_force(pos);
        f += 10 * collision_avoidance_force(pos);
        f += leader_force(pos);
        return f;
    }

    TV cohesion_separation_force(const TV pos) {
        TV f;
        f += cohesion_force(pos);
        f += separation_force(pos);
        return f / 2;
    }

    TV adversarial_strategy_1(const TV pos) {
        if(only_1_team && (*it_t) == false) {
            return stay_in_canvas_force(pos);
        }
        double mean_radius = (creation_radius + destruction_radius) /2;
        double mean_radius_squared = mean_radius * mean_radius;
        int nb_enemies = 0, nb_allies = 0;
        TV ally_barycenter, enemy_barycenter;

        //Iterate over each neighbor
        auto itt = teams.begin();
        for(auto itp = positions.begin(), itv = velocities.begin(); itp != positions.end(); itp++, itv++, itt++) {
            if(*itp == pos) continue; //same boid so skip
            double dist_squared = (pos - *itp).squaredNorm();
            if(*itt == *it_t) { //if same team
                if(dist_squared <= mean_radius_squared) {
                    ally_barycenter += *itp;
                    nb_allies++;
                }
            } else { //opposite team
                if(dist_squared <= mean_radius_squared) {
                    enemy_barycenter += *itp;
                    nb_enemies++;
                }
            }
        }
        TV f;
        if(nb_allies > 0) {
            ally_barycenter /= nb_allies;
            f += ally_barycenter - pos;
        }
        if(nb_enemies > 0) {
            enemy_barycenter /= nb_enemies;
            f += pos - enemy_barycenter;
        }
        f *= 10;
        f += stay_in_canvas_force(pos);
        return f;
    }

    TV adversarial_strategy_2(const TV pos) {
        if(only_1_team && (*it_t) == false) {
            return stay_in_canvas_force(pos);
        }
        double mean_radius = 0.15;
        double mean_radius_squared = mean_radius * mean_radius;
        int nb_enemies = 0, nb_allies = 0;
        TV ally_barycenter, enemy_barycenter;

        //Iterate over each neighbor
        auto itt = teams.begin();
        for(auto itp = positions.begin(), itv = velocities.begin(); itp != positions.end(); itp++, itv++, itt++) {
            if(*itp == pos) continue; //same boid so skip
            double dist_squared = (pos - *itp).squaredNorm();
            if(*itt == *it_t) { //if same team
                if(dist_squared <= mean_radius_squared) {
                    ally_barycenter += *itp;
                    nb_allies++;
                }
            } else { //opposite team
                if(dist_squared <= mean_radius_squared) {
                    enemy_barycenter += *itp;
                    nb_enemies++;
                }
            }
        }
        TV f;
        if(nb_allies > 0) {
            ally_barycenter /= nb_allies;
        }
        if(nb_enemies > 0) {
            enemy_barycenter /= nb_enemies;
            if(nb_allies > nb_enemies) { //number advantage => attack
                f += enemy_barycenter - pos;
            } else { //number disadvantage => flee
                f += pos - enemy_barycenter;
                f += ally_barycenter - pos;
            }
        }
        f *= 1.5;
        f += stay_in_canvas_force(pos);
        return f;
    }

    TV adversarial_strategy_3(const TV pos) {
        if(only_1_team && (*it_t) == false) {
            return stay_in_canvas_force(pos);
        }
        double mean_radius = 0.15;
        double mean_radius_squared = mean_radius * mean_radius;
        int nb_enemies = 0, nb_allies = 0, total_nb_enemies = 0;
        TV ally_barycenter, enemy_barycenter, total_enemy_barycenter;

        //Iterate over each neighbor
        auto itt = teams.begin();
        for(auto itp = positions.begin(), itv = velocities.begin(); itp != positions.end(); itp++, itv++, itt++) {
            if(*itp == pos) continue; //same boid so skip
            double dist_squared = (pos - *itp).squaredNorm();
            if(*itt == *it_t) { //if same team
                if(dist_squared <= mean_radius_squared) {
                    ally_barycenter += *itp;
                    nb_allies++;
                }
            } else { //opposite team
                if(dist_squared <= mean_radius_squared) {
                    enemy_barycenter += *itp;
                    nb_enemies++;
                }
                total_enemy_barycenter += *itp;
                total_nb_enemies++;
            }
        }
        TV f;
        if(nb_allies > 0) {
            ally_barycenter /= nb_allies;
        }
        if(nb_enemies > 0) {
            enemy_barycenter /= nb_enemies;
            if(nb_allies > 3 * nb_enemies / 2) { //clear number advantage => attack
                f += enemy_barycenter - pos;
            } else { //no clear number advantage => flee
                f += pos - enemy_barycenter;
                f += ally_barycenter - pos;
            }
        } else {
            if(total_nb_enemies > 0) { //no enemies in sight so move in direction
                                       //maximizing the prob of finding enemies
                total_enemy_barycenter /= total_nb_enemies;
                f += (total_enemy_barycenter - pos) / 2;
            }
        }
        f += stay_in_canvas_force(pos);
        return f;
    }

    TV adversarial_strategy_4(const TV pos) {
        if(only_1_team && (*it_t) == false) {
            return stay_in_canvas_force(pos);
        }
        double mean_radius = 0.15;
        double mean_radius_squared = mean_radius * mean_radius;
        int nb_enemies = 0, nb_allies = 0, total_nb_enemies = 0;
        TV ally_barycenter, enemy_barycenter, total_enemy_barycenter;

        //Iterate over each neighbor
        auto itt = teams.begin();
        for(auto itp = positions.begin(), itv = velocities.begin(); itp != positions.end(); itp++, itv++, itt++) {
            if(*itp == pos) continue; //same boid so skip
            double dist_squared = (pos - *itp).squaredNorm();
            if(*itt == *it_t) { //if same team
                if(dist_squared <= mean_radius_squared) {
                    ally_barycenter += *itp;
                    nb_allies++;
                }
            } else { //opposite team
                if(dist_squared <= mean_radius_squared) {
                    enemy_barycenter += *itp;
                    nb_enemies++;
                }
                total_enemy_barycenter += *itp;
                total_nb_enemies++;
            }
        }
        int total_nb_allies = positions.size() - total_nb_enemies;
        TV f;
        if(nb_allies > 0) {
            ally_barycenter /= nb_allies;
        }
        if(nb_enemies > 0) {
            enemy_barycenter /= nb_enemies;
            if(total_nb_enemies < total_nb_allies) { //global number advantage
                if(nb_allies > 3 * nb_enemies / 2 && total_nb_enemies) { //clear local number advantage => attack
                    f += enemy_barycenter - pos;
                } else { //no clear number advantage => flee
                    f += pos - enemy_barycenter;
                    f += ally_barycenter - pos;
                }
            } else {
                if(nb_allies > 0)
                    f += 2 * (ally_barycenter - pos);
            }
        } else {
            if(total_nb_enemies > 0) { //no enemies in sight so move in direction
                                       //maximizing the prob of finding enemies
                total_enemy_barycenter /= total_nb_enemies;
                f += (total_enemy_barycenter - pos) / 2;
            }
        }
        f += stay_in_canvas_force(pos);
        return f;
    }

    //Avoid the particle drifting away to infinity
    TV stay_in_canvas_force(const TV pos) {
        double distance = canvas_radius - pos.norm();
        TV f = - collision_avoidance_strength * pos.normalized() / (distance * distance);
        double dot = std::max(0.1, f.normalized().dot(-(*it_v).normalized()));
        return f * dot;
    }

    /*
    The 2 new rules (birth and death)
    */
    void life_cycle() {
        double creation_ref = creation_radius * creation_radius;
        double destruction_ref = destruction_radius * destruction_radius;

        //Iterate over each boid
        for(it_p = positions.begin(), it_v = velocities.begin(), it_t = teams.begin(), it_c = cooldowns.begin();
            it_p != positions.end(); it_p++, it_v++, it_t++, it_c++) {
            int nb_enemies = 0, nb_allies = 0;

            //Iterate over each neighbor
            auto itt = teams.begin();
            auto itc = cooldowns.begin();
            for(auto itp = positions.begin(), itv = velocities.begin(); itp != positions.end(); itp++, itv++, itt++, itc++) {
                if(itp == it_p) continue;
                double dist_squared = (*it_p - *itp).squaredNorm();
                if(*itt == *it_t) { //if same team
                    if(*it_c == 0 && *itc == 0) { //if cooldown of both boids is zero
                        if(dist_squared <= creation_ref) {
                            //Rule 1
                            positions.push_front((*it_p + *itp)/2);
                            velocities.push_front(-(*it_v + *itv)/2 + 0.1 * rand_TV()); //make the created boid go in opposite direction to avoid cascading effect
                            teams.push_front(*it_t); //the created boid is of the same team as the 'parents'
                            cooldowns.push_front(0);
                            nb_allies++;
                            (*it_c) = cooldown_delay;
                            (*itc) = cooldown_delay;
                        }
                    }
                } else {
                    if(dist_squared <= destruction_ref) {
                        nb_enemies++;
                    }
                }
            }
            //Rule 2
            if(*it_c > 0) --(*it_c); //reduce cooldown by 1 unit
            if(nb_allies == 0 && nb_enemies >= 3) { //die
                it_p = positions.erase(it_p); //points to next element
                it_v = velocities.erase(it_v);
                it_t = teams.erase(it_t);
                it_c = cooldowns.erase(it_c);
                it_p--; //backtrack because we want to process the element
                it_v--;
                it_t--;
                it_c--;
            }
        }
    }

    void updateBehavior(MethodTypes type, IntegrationMethods method) {
        if(!update)
            return;
        auto force_function = &Boids::const_downward_force;
        switch (type) {
            case FREEFALL:
                force_function = &Boids::const_downward_force;
                break;
            case ROTATION:
                force_function = &Boids::central_force;
                break;
            case COHESION:
                force_function = &Boids::cohesion_force;
                break;
            case ALIGNMENT:
                force_function = &Boids::alignment_force_2;
                break;
            case SEPARATION:
                force_function = &Boids::separation_force;
                break;
            case COLLISION_AVOIDANCE:
                force_function = &Boids::collision_avoidance_force;
                break;
            case LEADER:
                force_function = &Boids::additive_force;
                break;
            case TEAMS:
                force_function = &Boids::cohesion_separation_force; //cohesion force to increase the amount of interactions
                break;
            case COLLAB_ADV:
                force_function = &Boids::adversarial_strategy_4;
                break;
            default:
                break;
        }

        std::list<TV> positions_plus;
        std::list<TV> velocities_plus;
        for(it_p = positions.begin(), it_v = velocities.begin(), it_t = teams.begin();
            it_p != positions.end();
            it_p++, it_v++, it_t++) {
            time_integration(method, force_function, positions_plus, velocities_plus);
        }
        positions = positions_plus;
        velocities = velocities_plus;

        if(type == TEAMS || type == COLLAB_ADV) {
            life_cycle();
        }
    }

    template<typename Function>
    void time_integration(IntegrationMethods method, Function force_function,
        std::list<TV>& positions_plus, std::list<TV>& velocities_plus) {
        double m = 1; //mass
        double m_inv = 1 / m;
        TV x = *it_p;
        TV v = *it_v;
        TV x_plus, v_plus;

        //Update the position and velocity
        switch (method) {
            case SYMPLECTIC_EULER:
                symplecticEuler(x, v, m_inv, force_function, x_plus, v_plus);
                break;
            case EXPLICIT_EULER:
                explicitEuler(x, v, m_inv, force_function, x_plus, v_plus);
                break;
            case EXPLICIT_MIDPOINT:
                explicitMidpoint(x, v, m_inv, force_function, x_plus, v_plus);
                break;
            default:
                break;
        }
        
        //Set the new position and velocity
        positions_plus.push_back(x_plus);
        velocities_plus.push_back(v_plus);
    }

    template<typename Function>
    void explicitEuler(const TV& x, const TV& v, const double& m_inv, Function f, TV& x_plus, TV& v_plus) {
        x_plus = x + h * v;
        v_plus = v + h * m_inv * (this->*f)(x);
    }

    template<typename Function>
    void symplecticEuler(const TV& x, const TV& v, const double& m_inv, Function f, TV& x_plus, TV& v_plus) {
        x_plus = x + h * v;
        v_plus = v + h * m_inv * (this->*f)(x_plus);
    }

    template<typename Function>
    void explicitMidpoint(const TV& x, const TV& v, const double& m_inv, Function f, TV& x_plus, TV& v_plus) {
        TV x_half = x + h * v / 2;
        TV v_half = v + h * m_inv * (this->*f)(x) / 2;
        x_plus = x + h * v_half;
        v_plus = v + h * m_inv * (this->*f)(x_half);
    }

    void setLeaderPosition(TV pos) {
        leader_pos = pos;
    }

    double getBoid0Radius() {
        return positions.front().norm();
    }

    void pause() {
        update = !update;
    }

    std::list<TV> getPositions() {
        return positions;
    }

    std::list<bool> getTeams() {
        return teams;
    }

};
#endif

/**
 * @brief 
 * red
 * 
 */