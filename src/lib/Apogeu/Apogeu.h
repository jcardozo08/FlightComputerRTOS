#ifndef APOGEU_H
#define APOGEU_H

#include <Arduino.h>

class Apogeu {
  const int historicoTamanho;
  const float fatorDescida;
  float* historico;
  int indice = 0;
  bool estaDescendo;
  float estaDescendoHeight;
  unsigned long estaDescendoTime;
  bool firstApogeu = false;
  float maxHeight = 0;

public:
  Apogeu(int tamanho = 10, float fator = 0.6f);
  ~Apogeu();

  bool deteccaoApogeu(float entrada, short tipo);
  bool getEstaDescendo() const;
  float getApogeuHeight();
  unsigned long getApogeuTime();
  void maxAltitude(float altitude);
  float getMaxAltitude();

private:

bool metodo1();
bool metodo2();
bool metodo3();


};

#endif
