---
layout: post
title:  "Codigo fonte e resultado"
date:   2023-10-27 00:00:52 -0200
author: Jonatas S.
permalink: /fontCode
---

![Imagem do Robo][{{site.url}}(/assets/Robot.jpg)]


`Este foi o design final do robo, oque agradou a todos os membros do grupo.`

 

Aqui temos o codigo fonte utilizado para o movimento dele:

{% highlight INO %}
#include <SparkFun_TB6612.h>

//definição dos pinos
#define AIN1 11
#define BIN1 9
#define AIN2 12
#define BIN2 8
#define PWMA 5
#define PWMB 7
#define STBY 10

// constantes para configurar os motores invertendo seu sentido caso seja necessário
const int inversorA = 1;
const int inversorB = 1;

// Inicialização dos motores.
Motor motor1 = Motor(BIN1, BIN2, PWMB, inversorA, STBY);
Motor motor2 = Motor(AIN1, AIN2, PWMA, inversorB, STBY);


// Definição das variaveis para o controle PDI
int P{}, D{}, I{}, erroPrevio{}, PID{}, erro{};

// Definição das variaveis para o ajuste da velocidade dos motores
int veloE{}, veloD{};

// Variável da velocidade base com que o robô segue a linha
int veloR{210};


// Constantes de controle do PDI, para ajustar o comportamento do robô
float Kp{0}, Kd{0}, Ki{0};

// Matrizes para a calibração dos sensores
int minValues[6]{}, maxValues[6]{}, limite[6]{};


/* Função de calibração dos sensores.
   Onde é feita a coleta de dados para ajustar
   o robô e sua direção
*/
void calibragemS()
{
  for ( int i = 8; i < 13; i++)
  {
    int y = i-7;
    minValues[y] = analogRead(i);
    maxValues[y] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    motor1.drive(200);
    motor2.drive(-220);

    for ( int i = 8; i < 13; i++)
    {
      int y = i-7;
      if (analogRead(i) < minValues[y])
      {
        minValues[y] = analogRead(i);
      }
      if (analogRead(i) > maxValues[y])
      {
        maxValues[y] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 6; i++)
  {
    limite[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(limite[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}

/*
  Aplicação do controle PID para o ajuste da rota
*/
void linefollow()
{
  int erro = (analogRead(9) - analogRead(11));

  P = erro;
  I = I + erro;
  D = erro - erroPrevio;

  PID = (Kp * P) + (Ki * I) + (Kd * D);
  erroPrevio = erro;

  veloE = veloR - PID;
  veloD = veloR + PID;

  if (veloE > 255) {
    veloE = 255;
  }
  if (veloE < 0) {
    veloE = 0;
  }
  if (veloD > 255) {
    veloD = 255;
  }
  if (veloD < 0) {
    veloD = 0;
  }
  motor1.drive(veloE);
  motor2.drive(veloD);

}

// Código de configuração dos botões
void setup()
{
  Serial.begin(9600);
  // Entradas como resistores pull-up
  pinMode(23, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);
}


//
void loop()
{
  
  delay(1000);
  while (digitalRead(25)) {}
  delay(1000);
  calibragemS();
  delay(2000);
  while (digitalRead(23)) {}
  //delay(1000);

  while (1)
  {
    if (analogRead(8) > limite[1] && analogRead(12) < limite[5] )
    {
      veloE = 0; veloD = veloR;
      motor1.drive(0);
      motor2.drive(veloR);
    }

    else if (analogRead(12) > limite[5] && analogRead(8) < limite[1])
    { veloE = veloR; veloD = 0;
      motor1.drive(veloR);
      motor2.drive(0);
    }
    else if (analogRead(10) > limite[3])
    {
      Kp = 0.0003 * (1000 - analogRead(10));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  }
}


{% endhighlight %}