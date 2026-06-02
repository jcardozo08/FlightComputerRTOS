#ifndef PARAQUEDAS_H
#define PARAQUEDAS_H

class Paraquedas {
  const int intervaloLigado;
  bool paraquedas = false;
  bool paraquedasData = false;
  unsigned long paraquedasTempo = 0;
  const int ign;
  const int saude;
  float delay;
  float altura;
  int sensibilidadeSaude;
  unsigned long timePqd = 0;
  float alturaPqd = 0;

public:
  Paraquedas(int tempoLigado, float tempoDelay, int portaIgn, int portaSaude, float alturaAtivacao, int sensibilidade = 100);

  void begin();
  void ativarParaquedas(float alturaAtual, unsigned long currentTime, bool estaDescendo, bool estaDescendoTimer = 0);
  bool getData() const;
  bool getInfo();
  unsigned long getPqdTime();
  float getPqdHeight();
};

#endif
