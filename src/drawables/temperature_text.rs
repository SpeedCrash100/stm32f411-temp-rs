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
        let sign_count = 2;
        let coff = 10_i32.pow(sign_count);
        let rounded = (temperature * coff as f32).round() as i32;

        let integer_part = rounded / coff;
        let fract_part = (rounded % coff).abs();

        str.clear();

        if fract_part < 10 {
            uwrite!(str, "{}.0{}", integer_part, fract_part).ok();
        } else {
            uwrite!(str, "{}.{}", integer_part, fract_part).ok();
        }
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
