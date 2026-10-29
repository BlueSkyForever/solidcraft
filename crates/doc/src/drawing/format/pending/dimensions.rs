//! Drawings save their dimensions (`dimensions`, #4896 W19b): each on one of the drawing's views,
//! measuring named corners, circles' and arcs' centres, or edges of the view's model (a linear
//! distance, an edge's length, an angle, a circle's or an arc's radius or diameter, or a chain of
//! distances along one line), with how far its line sits from them and where its text sits on
//! paper, its decimals and any text shown instead of its number. An older drawing has no
//! dimensions. The last release refuses a drawing saved with them, as a newer format.
