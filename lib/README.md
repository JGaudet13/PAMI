# PIDLineFollower - Classe de Suivi de Ligne

Classe modulaire pour le contrôle d'un robot suiveur de ligne utilisant un système de PID en cascade.

## Architecture

La classe implémente un système de contrôle en deux niveaux :

1. **PID de Suivi de Ligne** (haut niveau)
   - Entrée : Erreur du capteur de ligne
   - Sortie : Vitesses cibles pour les moteurs
   - Rôle : Diriger le robot vers la ligne

2. **PID des Moteurs** (bas niveau)
   - Entrée : Erreur entre vitesse cible et vitesse réelle
   - Sortie : Correction PWM appliquée aux moteurs
   - Rôle : Réguler précisément la vitesse de chaque moteur

## Utilisation

### Initialisation

```cpp
#include "PIDLineFollower.h"

// Créer l'instance
PIDLineFollower line_follower(&motor_right, &motor_left, &sensors);

// Initialiser
line_follower.init();
```

### Configuration

```cpp
// Vitesse de base (0.0 à 1.0)
line_follower.setBaseSpeed(0.5);

// Limites de vitesse
line_follower.setSpeedLimits(-1.0, 1.0);

// Gains PID du suivi de ligne
line_follower.setLineFollowerGains(0.8, 0.1, 0.5);  // Kp, Ki, Kd

// Gains PID des moteurs
line_follower.setMotorGains(1.5, 0.3, 0.2);        // Kp, Ki, Kd
```

### Contrôle

```cpp
// Dans loop()
line_follower.followLine();  // Suit automatiquement la ligne
```

### Méthodes Utiles

```cpp
line_follower.stop();       // Arrêter les moteurs
line_follower.reset();      // Réinitialiser les PID
```

### Debug

```cpp
// Valeurs pour le debug
double line_error = line_follower.getLineError();
double target_right = line_follower.getRightTargetSpeed();
double actual_right = line_follower.getRightActualSpeed();
double pwm_right = line_follower.getRightPWM();
// ... même chose pour le moteur gauche
```

## Ajustement des Gains PID

### PID Suivi de Ligne
- **Kp** : Réactivité à l'erreur de ligne (0.5 - 1.5)
- **Ki** : Correction des erreurs persistantes (0.0 - 0.2)
- **Kd** : Réduction des oscillations (0.2 - 0.8)

### PID Moteurs
- **Kp** : Réactivité à l'erreur de vitesse (1.0 - 2.0)
- **Ki** : Correction des écarts persistants (0.1 - 0.5)
- **Kd** : Stabilité (0.1 - 0.3)

## Exemple Complet

Voir `src/main.cpp` pour un exemple d'utilisation complète.

## Dépannage

- **Robot oscille** : Réduire Kp du PID de suivi de ligne
- **Robot ne réagit pas** : Augmenter Kp du PID de suivi de ligne
- **Vitesses instables** : Ajuster les gains du PID moteur
- **Robot trop lent** : Augmenter la vitesse de base