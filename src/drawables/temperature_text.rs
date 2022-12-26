use embedded_graphics::{
    prelude::*,
    text::{renderer::TextRenderer, Text},
    Drawable,
};
use heapless::String;
use num_traits::Float;
use ufmt::uwrite;

const TEMPERATURE_STRING_SIZE: usize = 7; // (+|-)\d{0,3}\.\d{0,2}

pub struct TemperatureText<Style> {
    temperature_string: String<TEMPERATURE_STRING_SIZE>,
    style: Style,
    position: Point,
}

impl<Style> TemperatureText<Style> {
    pub fn new(position: Point, style: Style) -> Self {
        let temp: f32 = 0.0;
        let mut temperature_string = String::new();

        Self::write_temp(temp, &mut temperature_string);

        Self {
            temperature_string,
            style,
            position,
        }
    }

    pub fn with_temperature(self, temperature: f32) -> Self {
        let mut temperature_string = String::new();
        Self::write_temp(temperature, &mut temperature_string);

        Self {
            temperature_string,
            ..self
        }
    }

    fn write_temp<const N: usize>(temperature: f32, str: &mut String<N>) {
        let integer_part = temperature.floor();
        let fract_part = ((temperature - integer_part) * 100.0).round().abs();

        str.clear();

        uwrite!(str, "{}.{}", integer_part as i32, fract_part as i32).ok();
    }
}

impl<Color, Style> Drawable for TemperatureText<Style>
where
    Color: PixelColor,
    Style: TextRenderer<Color = Color> + Clone,
{
    type Color = Color;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        let text = Text::new(&self.temperature_string, self.position, self.style.clone());
        text.draw(target)?;
        Ok(())
    }
}
