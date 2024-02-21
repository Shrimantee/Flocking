#include <SDL.h>
#include <vector>
#include <cmath>

// Constants
const int WIDTH = 800;
const int HEIGHT = 600;
const int NUM_BOIDS = 100;
const float MAX_SPEED = 3.0f;
const float MAX_FORCE = 0.1f;
const float DESIRED_SEPARATION = 25.0f;
const float NEIGHBOR_RADIUS = 50.0f;

// 2D Vector
struct Vector2D {
    float x, y;

    Vector2D() : x(0), y(0) {}

    Vector2D(float _x, float _y) : x(_x), y(_y) {}

    float magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    void normalize() {
        float mag = magnitude();
        if (mag > 0) {
            x /= mag;
            y /= mag;
        }
    }

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }
    Vector2D operator/(float scalar) const {
        return Vector2D(x / scalar, y / scalar);
    }
};

// Boid class
class Boid {
public:
    Vector2D position;
    Vector2D velocity;

    Boid(float x, float y) : position(x, y), velocity((rand() % 3 - 1.5f), (rand() % 3 - 1.5f)) {}

    void update(const std::vector<Boid>& boids) {
        Vector2D separation = separate(boids);
        Vector2D alignment = align(boids);
        Vector2D cohesionForce = cohesion(boids);

        Vector2D acceleration = separation + alignment + cohesionForce;

        velocity = velocity + acceleration;
        velocity.normalize();
        velocity = velocity * MAX_SPEED;

        position = position + velocity;

        // Wrap around screen
        if (position.x < 0) position.x = WIDTH;
        if (position.x > WIDTH) position.x = 0;
        if (position.y < 0) position.y = HEIGHT;
        if (position.y > HEIGHT) position.y = 0;
    }

private:
    Vector2D separate(const std::vector<Boid>& boids) {
        Vector2D steer(0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = (position - other.position).magnitude();
            if (d > 0 && d < DESIRED_SEPARATION) {
                Vector2D diff = position - other.position;
                diff.normalize();
                diff = diff / d;
                steer = steer + diff;
                count++;
            }
        }
        if (count > 0) {
            steer = steer / static_cast<float>(count);
        }

        if (steer.magnitude() > 0) {
            steer.normalize();
            steer = steer * MAX_SPEED;
            steer = steer - velocity;
            if (steer.magnitude() > MAX_FORCE) {
                steer.normalize();
                steer = steer * MAX_FORCE;
            }
        }
        return steer;
    }

    Vector2D align(const std::vector<Boid>& boids) {
        Vector2D sum(0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = (position - other.position).magnitude();
            if (d > 0 && d < NEIGHBOR_RADIUS) {
                sum = sum + other.velocity;
                count++;
            }
        }
        if (count > 0) {
            sum = sum / static_cast<float>(count);
            sum.normalize();
            sum = sum * MAX_SPEED;

            Vector2D steer = sum - velocity;
            if (steer.magnitude() > MAX_FORCE) {
                steer.normalize();
                steer = steer * MAX_FORCE;
            }
            return steer;
        }
        else {
            return Vector2D(0, 0);
        }
    }

    Vector2D cohesion(const std::vector<Boid>& boids) {
        Vector2D sum(0, 0);
        int count = 0;
        for (const auto& other : boids) {
            float d = (position - other.position).magnitude();
            if (d > 0 && d < NEIGHBOR_RADIUS) {
                sum = sum + other.position;
                count++;
            }
        }
        if (count > 0) {
            sum = sum / static_cast<float>(count);
            return seek(sum);
        }
        else {
            return Vector2D(0, 0);
        }
    }

    Vector2D seek(const Vector2D& target) {
        Vector2D desired = target - position;
        desired.normalize();
        desired = desired * MAX_SPEED;

        Vector2D steer = desired - velocity;
        if (steer.magnitude() > MAX_FORCE) {
            steer.normalize();
            steer = steer * MAX_FORCE;
        }
        return steer;
    }
};

int main() {
    SDL_Window* window = NULL;
    SDL_Renderer* renderer = NULL;

    /*if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }*/

    window = SDL_CreateWindow("Flocking Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        //std::cerr << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        //std::cerr << "Renderer could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    std::vector<Boid> boids;
    for (int i = 0; i < NUM_BOIDS; ++i) {
        boids.emplace_back(rand() % WIDTH, rand() % HEIGHT);
    }

    bool quit = false;
    while (!quit) {
        SDL_Event e;
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            }
        }

        // Update boids
        for (auto& boid : boids) {
            boid.update(boids);
        }

        // Clear the renderer
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Draw boids
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        for (const auto& boid : boids) {
            SDL_Rect rect = { static_cast<int>(boid.position.x), static_cast<int>(boid.position.y), 3, 3 };
            SDL_RenderFillRect(renderer, &rect);
        }

        // Render the scene
        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}
