use bevy::prelude::*;
use rand::distr::weighted::WeightedIndex;
use rand::distr::Distribution;
use rand::Rng;

// A simple struct to create a probability distribution
#[derive(Resource)]
pub struct WeightedProbabilityDistribution {
    values: Vec<f64>,
    weighted_index: WeightedIndex<f64>,
}

impl WeightedProbabilityDistribution {
    pub fn new(values: Vec<f64>, weights: Vec<f64>) -> Self {
        WeightedProbabilityDistribution {
            values,
            weighted_index: WeightedIndex::new(&weights).unwrap(),
        }
    }
}

impl Distribution<f64> for WeightedProbabilityDistribution {
    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> f64 {
        let index = self.weighted_index.sample(rng);
        self.values[index]
    }
}
