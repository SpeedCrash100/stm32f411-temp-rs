use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Line, PrimitiveStyle};
use num_traits::Float;

const FLOAT_TOLLERANCE: f32 = 0.00001;

/// Basic plot definition without data
pub struct Plot<Color: PixelColor> {
    style: PrimitiveStyle<Color>,
    position: Point,
    size: Size,
}

impl<Color: PixelColor> Plot<Color> {
    pub fn new(style: PrimitiveStyle<Color>) -> Self {
        Self {
            style,
            position: Point::new(0, 0),
            size: Size::new(32, 32),
        }
    }

    pub fn with_position(self, position: Point) -> Self {
        Self { position, ..self }
    }

    pub fn with_size(self, size: Size) -> Self {
        Self { size, ..self }
    }

    /// Generates plot object with data that can be drawn
    pub fn generate<'plot, 'data: 'plot>(
        &'plot self,
        data: &'data [f32],
    ) -> PlotData<'_, '_, Color> {
        PlotData { plot: self, data }
    }

    fn normalize_point_y(y: f32, min: f32, max: f32) -> f32 {
        (y - min) / (max - min)
    }

    fn display_point_y(&self, y: f32, min: f32, max: f32) -> i32 {
        let y_norm = 1.0 - Self::normalize_point_y(y, min, max);
        let y_scaled = y_norm * (self.size.height as f32 - 1.0);

        self.position.y + y_scaled.round() as i32
    }
}

/// Plot with data that ready to draw
pub struct PlotData<'plot, 'data: 'plot, Color: PixelColor> {
    plot: &'plot Plot<Color>,
    data: &'data [f32],
}

impl<'plot, 'data: 'plot, Color: PixelColor> Drawable for PlotData<'plot, 'data, Color> {
    type Color = Color;
    type Output = ();

    fn draw<D>(&self, target: &mut D) -> Result<Self::Output, D::Error>
    where
        D: DrawTarget<Color = Self::Color>,
    {
        let point_count = self.data.len() as u32;
        let step = self.plot.size.width as f32 / point_count as f32;

        let max_data = self.data.iter().cloned().reduce(f32::max).unwrap();
        let min_data = self.data.iter().cloned().reduce(f32::min).unwrap();
        let range = max_data - min_data;

        let position = self.plot.position;
        let size = self.plot.size;

        // Range too low. We have to just draw stright line
        if range < FLOAT_TOLLERANCE {
            // Draw a straight line
            let center_y = (size.height / 2) as i32;

            Line::new(
                Point::new(position.x, position.y + center_y),
                Point::new(position.x + size.width as i32 - 1, position.y + center_y),
            )
            .into_styled(self.plot.style)
            .draw(target)?;

            return Ok(());
        }
        // Else draw normal plot

        // Create iterator with data pairs (i, i+1)
        let iterator_wo_last = self.data[..(self.data.len() - 1)].iter();
        let iterator_wo_start = self.data.iter().skip(1);
        let iterator_pairs = iterator_wo_last.zip(iterator_wo_start);

        for (i, (start_data, end_data)) in iterator_pairs.enumerate() {
            let x_start = position.x + (step * i as f32).round() as i32;
            let x_end = position.x + (step * (i + 1) as f32).round() as i32;
            let y_start = self.plot.display_point_y(*start_data, min_data, max_data);
            let y_end = self.plot.display_point_y(*end_data, min_data, max_data);

            Line::new(Point::new(x_start, y_start), Point::new(x_end, y_end))
                .into_styled(self.plot.style)
                .draw(target)?;
        }

        Ok(())
    }
}
