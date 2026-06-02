#include "Apogeu.h"
#include <stdlib.h> // Para malloc e free

Apogeu::Apogeu(int tamanho, float fator)
  : historicoTamanho(tamanho), fatorDescida(fator) {
  historico = (float*)malloc(historicoTamanho * sizeof(float));
  if (historico == NULL) {

  }
  for (int i = 0; i < historicoTamanho; i++) {
    historico[i] = 0; // Inicializa o array com zeros
  }
}

Apogeu::~Apogeu() {
  // Libera a memória alocada dinamicamente
  free(historico);
}

bool Apogeu::deteccaoApogeu(float entrada, short tipo) {
  historico[indice] = entrada;
  int contadorHistorico = 0;
  
  if (tipo == 0) {
    for (int i = 1; i < historicoTamanho; i++) {
      if (historico[(indice + i - 1) % historicoTamanho] > historico[(indice + i) % historicoTamanho]) {
        contadorHistorico++;
      }
   }

   estaDescendo = false;

    if (contadorHistorico >= fatorDescida * historicoTamanho) {
      estaDescendo = true;
      if (!firstApogeu) {
        estaDescendoHeight = entrada;
        estaDescendoTime = millis();
        firstApogeu = true;
      }
    }
  }
  else {
    for (int i = 0; i < historicoTamanho; i++) {
      if (indice != i) {
        if (historico[(indice)] < historico[(i)]-0.03) {
        contadorHistorico++;
        }
      }
    }

    estaDescendo = false;

  
    if (contadorHistorico == historicoTamanho-1) {
      estaDescendo = true;
      if (!firstApogeu) {
        estaDescendoHeight = entrada;
        estaDescendoTime = millis();
        firstApogeu = true;
      }
    }
  }

  if (++indice >= historicoTamanho) {
    indice = 0;
  }

  return estaDescendo;
}

void Apogeu::maxAltitude(float altitude) {
  if (altitude >= maxHeight) {
    maxHeight = altitude;
  }
}

bool Apogeu::getEstaDescendo() const {
  return estaDescendo;
}

float Apogeu::getApogeuHeight() {
  return estaDescendoHeight;
}

unsigned long Apogeu::getApogeuTime() {
  return estaDescendoTime;
}

float Apogeu::getMaxAltitude() {
  return maxHeight;
}