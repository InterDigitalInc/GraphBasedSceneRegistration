#include "utils.hpp"

char char2dtype(const char c) {
  switch (c) {
    case 'R': case 'r':
      return WALK;
    case 'F': case 'f':
      return FAST;
    case 'G': case 'g':
      return FAS2;
    case 'U': case 'u':
      return NEWD;
    case 'A':// case 'a':
      return ADJ_;
    case 'B':// case 'b':
      return BDJ_;
    case 'a'://case 'M': case 'm':
      return ADJN;
    case 'b'://case 'N': case 'n':
    default:
      return BDJN;
    case 'C'://case 'O': case 'o':
      return CDJ_;
    case 'c'://case 'P': case 'p':
      return CDJN;
  }
}

char dtype2char(const char type) {
  switch (type) {
    case WALK:
      return 'r';
    case FAST:
      return 'f';
    case FAS2:
      return 'g';
    case NEWD:
      return 'u';
    case ADJ_:
      return 'A';//return 'a';
    case BDJ_:
      return 'B';//return 'b';
    case ADJN:
      return 'a';//return 'm';
    case BDJN:
    default:
      return 'b';//return 'n';
    case CDJ_:
      return 'C';//return 'o';
    case CDJN:
      return 'c';//return 'p';
  }
}

char char2ctype(const char c) {
  switch (c) {
    case 'F': case 'f':
    default:
      return FULLC;
    case 'M': case 'm':
      return MEANC;
    case 'P': case 'p':
      return PCA7C;
    case 'G': case 'g':
      return GNODE;
  }
}

char ctype2char(const char type) {
  switch (type) {
    case FULLC:
    default:
      return 'f';
    case MEANC:
      return 'm';
    case PCA7C:
      return 'p';
    case GNODE:
      return 'g';
  }
}

char char2rtype(const char c) {
  switch (c) {
    case 'N': case 'n':
      return NORANSAC;
    case 'A': case 'a':
      return ALLNODES;
    case 'S': case 's':
      return ONEPERSN;
    case 'T': case 't':
      return ONEPERTN;
    case 'M': case 'm':
    default:
      return ONEPERMN;
  }
}

char rtype2char(const char type) {
  switch (type) {
    case NORANSAC:
      return 'n';
    case ALLNODES:
      return 'a';
    case ONEPERSN:
      return 's';
    case ONEPERTN:
      return 't';
    case ONEPERMN:
    default:
      return 'm';
  }
}

char char2task(const char c) {
  switch (c) {
    case 'C':
      return CLOUDS;
    case 'B':
      return RBLOBS;
    case 'M': case 'm':
      return MODELS;
    case '1':
      return ALIGN1;
    case '2':
      return ALIGN2;
    case 'G': case 'g':
      return GTRUTH;
    case 'a':
      return ANIMAT;
    case 'c':
      return ANIMA2;
    case 'b':
      return ANIMA1;
    case 'P': case 'p':
      return POINTS;
    case 'E': case 'e':
      return TOPOLO;
    case 'T': case 't':
      return TESTGT;
    case 'R': case 'r':
      return TEASER;
    case 'F': case 'f':
      return FORCED;
    case 'H':
      return EDGES2;
    case 'h':
      return EDGES1;
    default:
      return NOPLOT;
  }
}

char char2task(const char c, const char d) {
  switch (c) {
    case 'C': case 'c':
      if (d == '1') return ALIGN1;
      if (d == '2') return ALIGN2;
      return CLOUDS;
    case 'M': case 'm':
      return MODELS;
    case 'G': case 'g':
      return GTRUTH;
    case 'A': case 'a':
      if (d == '1') return ANIMA1;
      if (d == '2') return ANIMA2;
      return ANIMAT;
    case 'P': case 'p':
      return POINTS;
    case 'E': case 'e':
      if (d == '1') return EDGES1;
      if (d == '2') return EDGES2;
      return TOPOLO;
    case 'T': case 't':
      return TESTGT;
    case 'R': case 'r':
      return TEASER;
    case 'F': case 'f':
      return FORCED;
    default:
      return NOPLOT;
  }
}
