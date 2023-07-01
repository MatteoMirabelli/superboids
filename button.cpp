#include "button.hpp"
#include <SFML/Graphics.hpp>

template <typename T>
// implement set position function
void Button<T>::setPosition(sf::Vector2f position)
{
    button_shape.setPosition(position);
    button_text.setPosition(position);
}

template <typename T>
void Button<T>::setBoxColor(sf::Color color)
{
    button_shape.setFillColor(color);
}

template <typename T>
void Button<T>::setTextColor(sf::Color color)
{
    button_text.setFillColor(color);
}

// function that return bool true if the cursor is on the button and left mouse button is pressed
template <typename T>
bool Button<T>::isButtonClicked(sf::RenderWindow &window)
{
    sf::Vector2f mouse_position = sf::Vector2f(sf::Mouse::getPosition(window));
    if (button_shape.getGlobalBounds().contains(mouse_position) && sf::Mouse::isButtonPressed(sf::Mouse::Left))
    {
        return true;
    }
    return false;
}