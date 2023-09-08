#include <SFML/Graphics.hpp>

class Visualisation
{
public:
    Visualisation();
    ~Visualisation() {}

    void update();

private:
    sf::RenderWindow window_;
    sf::CircleShape shape_;
};
