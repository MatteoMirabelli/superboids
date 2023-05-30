#ifndef BUTTON_HPP
#define BUTTON_HPP

#include <SFML/Graphics.hpp>
#include <SFML/System/String.hpp>
// #include <type_traits>
// #include <cassert>

template <typename T>
class Button : public sf::Drawable
{
    virtual void draw(sf::RenderTarget &target, sf::RenderStates states) const
    {
        // draw shape and text on top of it
        target.draw(button_shape, states);
        target.draw(button_text, states);
    }
    sf::RectangleShape button_shape;
    sf::Text button_text;
    T button_value;

public:
    // button constructor
    Button(T const &value, sf::Font const &font) : button_value(value)
    {
        // set button shape
        button_shape.setSize(sf::Vector2f(130.f, 50.f));
        button_shape.setFillColor(sf::Color(255, 255, 255, 0));
        button_shape.setOutlineColor(sf::Color::Black);
        button_shape.setOutlineThickness(3.f);
        // set button text
        button_text.setFont(font);
        // set text string to value if it is a string, else convert it to string
        // constexpr required to check at compile time if T is a string
        if constexpr (std::is_same_v<T, std::string>)
        {
            button_text.setString(value);
        }
        else
        {
            button_text.setString(std::to_string(value));
        }
        button_text.setCharacterSize(20);
        button_text.setFillColor(sf::Color::Black);
        // set text origin to its center
        button_text.setOrigin(button_shape.getLocalBounds().left + button_text.getLocalBounds().width / 2.0f,
                              button_text.getLocalBounds().top + button_text.getLocalBounds().height / 2.0f);
        // set shape origin to its center
        button_shape.setOrigin(button_shape.getLocalBounds().left + button_shape.getLocalBounds().width / 2.0f,
                               button_shape.getLocalBounds().top + button_shape.getLocalBounds().height / 2.0f);
        // set shape position to 150, 50
        button_shape.setPosition(65.f, 25.f);
        // align text origin to shape origin
        button_text.setPosition(button_shape.getPosition());
    }
    void setPosition(sf::Vector2f);
    void setBoxColor(sf::Color);
    void setTextColor(sf::Color);
    // function that return bool true if the cursor is on the button and left mouse button is pressed
    bool isButtonClicked(sf::RenderWindow &);
};

#endif