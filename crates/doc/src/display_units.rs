//! The existing length display policy, shared by windowed and headless consumers.
//!
//! Values are stored in millimetres and displayed in any length unit of the expression unit table
//! ([`UNITS`]): a design's five units and the others a drawing may give its measurements in. One
//! table for both, so every length unit a design or a drawing accepts shows its own numbers. Other
//! names retain the millimetre fallback. These functions format values, not validate them:
//! non-finite inputs retain Rust's `NaN`, `inf` and `-inf` text. Callers that require finite
//! measurements must validate them first.

use crate::expr::{Kind, UNITS};

/// Units per millimetre and the unit's name, for a length unit of [`UNITS`]. Other names use
/// millimetres.
pub fn unit_scale(units: &str) -> (f64, &'static str) {
    match UNITS.iter().find(|(name, kind, _)| *name == units && *kind == Kind::Length) {
        Some(&(name, _, mm)) => (1.0 / mm, name),
        None => (1.0, "mm"),
    }
}

/// Inch labels (`in` or `inch`) retain a meaningful fourth digit without promoting floating-point
/// noise. `value` is already converted to the requested display unit, not a millimetre value.
pub fn length_precision(value: f64, units: &str, decimals: u8) -> usize {
    let base = usize::from(decimals.min(8));
    if unit_scale(units).0 != unit_scale("in").0 {
        return base;
    }
    let needs_four = ((value * 10_000.0).round() - (value * 1000.0).round() * 10.0).abs() >= 0.5;
    base.max(if needs_four { 4 } else { 3 })
}

/// A millimetre length in the requested display unit, without a unit suffix.
pub fn length_number(mm: f64, units: &str, decimals: u8) -> String {
    let value = mm * unit_scale(units).0;
    format!("{:.*}", length_precision(value, units, decimals), value)
}

/// A length in millimetres in `units`, with `decimals` (for display).
pub fn format_length(mm: f64, units: &str, decimals: u8) -> String {
    let (_, u) = unit_scale(units);
    format!("{} {u}", length_number(mm, units, decimals))
}

#[cfg(test)]
mod tests {
    use super::{format_length, length_number, length_precision, unit_scale};
    use crate::expr::{Kind, UNITS};

    #[test]
    fn all_design_units_keep_bare_and_suffixed_labels() {
        for (mm, units, bare, suffixed) in [
            (12.5, "mm", "12.50", "12.50 mm"),
            (125.0, "cm", "12.50", "12.50 cm"),
            (12_500.0, "m", "12.50", "12.50 m"),
            (317.5, "in", "12.500", "12.500 in"),
            (3810.0, "ft", "12.50", "12.50 ft"),
        ] {
            assert_eq!(length_number(mm, units, 2), bare, "{units}");
            assert_eq!(format_length(mm, units, 2), suffixed, "{units}");
        }
    }

    #[test]
    fn inch_labels_keep_three_or_four_digits_without_promoting_noise() {
        for (mm, decimals, expected) in [
            (25.4, 0, "1.000"),
            (25.4 * 1.234, 2, "1.234"),
            (25.4 * 1.2345, 0, "1.2345"),
            (-25.4 * 1.2345, 3, "-1.2345"),
            (25.4 * 0.001, 2, "0.001"),
            (25.4 * (1.0 + 1e-12), 2, "1.000"),
            (25.4, 4, "1.0000"),
            (25.4, 5, "1.00000"),
            (0.0, 0, "0.000"),
        ] {
            assert_eq!(length_number(mm, "in", decimals), expected, "{mm} mm, {decimals} places");
        }
    }

    #[test]
    fn precision_uses_the_converted_value_and_caps_requested_places() {
        assert_eq!(length_precision(1.2345, "in", 2), 4);
        assert_eq!(length_precision(1.234, "in", 2), 3);
        assert_eq!(length_precision(1.2345, "mm", 2), 2);
        assert_eq!(length_precision(1.2345, "in", 6), 6);
        assert_eq!(length_precision(1.2345, "in", u8::MAX), 8);
        assert_eq!(length_precision(1.2345, "mm", u8::MAX), 8);
        assert_eq!(length_number(12.5, "mm", u8::MAX), "12.50000000");
        assert_eq!(length_number(25.4, "in", u8::MAX), "1.00000000");
    }

    #[test]
    fn rounding_and_signed_zero_keep_the_existing_display_policy() {
        for (mm, decimals, expected) in [
            (0.0, 2, "0.00"),
            (-0.0, 2, "-0.00"),
            (-0.0001, 2, "-0.00"),
            (-12.5, 2, "-12.50"),
            (12.5, 0, "12"),
            (13.5, 0, "14"),
            (-12.5, 0, "-12"),
            (-13.5, 0, "-14"),
            (1.25, 1, "1.2"),
            (1.75, 1, "1.8"),
        ] {
            assert_eq!(length_number(mm, "mm", decimals), expected, "{mm} mm, {decimals} places");
        }
        assert_eq!(format_length(-0.0, "in", 2), "-0.000 in");
    }

    #[test]
    fn every_length_unit_shows_its_own_numbers() {
        // Each length unit of the expression table, which a design's units and a drawing's unit
        // come from, never millimetres unless it is millimetres.
        for (name, kind, mm) in UNITS {
            if kind != Kind::Length {
                continue;
            }
            assert_eq!(unit_scale(name), (1.0 / mm, name), "{name}");
            let bare = if matches!(name, "in" | "inch") { "12.500" } else { "12.50" };
            assert_eq!(length_number(12.5 * mm, name, 2), bare, "{name}");
            assert_eq!(format_length(12.5 * mm, name, 2), format!("{bare} {name}"), "{name}");
        }
        assert_eq!(length_number(25.4 * 1.2345, "inch", 0), "1.2345");
    }

    #[test]
    fn unknown_display_units_keep_the_millimetre_fallback() {
        // Names that aren't a length unit of the expression table, angles included.
        for units in ["", " in ", "IN", "unsupported", "deg", "rad"] {
            assert_eq!(unit_scale(units), (1.0, "mm"), "{units}");
            assert_eq!(length_number(25.4, units, 2), "25.40", "{units}");
            assert_eq!(format_length(25.4, units, 2), "25.40 mm", "{units}");
        }
    }

    #[test]
    fn non_finite_text_is_preserved_without_claiming_validation() {
        assert_eq!(length_number(f64::NAN, "mm", 2), "NaN");
        assert_eq!(format_length(f64::INFINITY, "in", u8::MAX), "inf in");
        assert_eq!(format_length(f64::NEG_INFINITY, "cm", 2), "-inf cm");
    }
}
