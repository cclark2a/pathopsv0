#pragma once

// int value is (mostly) point count - 1 (conic is exception)
  // !!! eventually replace this with user extensible types
enum class OpType {
    no,
    line,
    quad,
    conic,
    cubic
};

// returns if an edge starts a fill, ends a fill, or does neither and should be discarded
enum class WindKeep {
    End,
    Start,
    Discard
};

