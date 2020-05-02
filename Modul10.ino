#include <Keypad.h>


/**
 * PushButton class declaration. Normally in a separate file.
 */
class PushButton
{
  private: 
    int pin;
    bool isDown;
    void (*onButtonDownEvent) (void);
  public:
    PushButton(int inputPin);
    void Update();
    void SetOnButtonDownEvent(void (*event)(void));
};

/**
 * The pin connecting the arduino to the buzzer.
 */
#define BUZZER_PIN 9

/**
 * Definitions of the pins connecting
 * the arduino with an RPG led.
 */
#define RED_PIN 6
#define GREEN_PIN 3
#define BLUE_PIN 5

/**
 * Definitions of the pins connecting
 * the arduino with the push buttons.
 */
#define BUT_0 4
#define BUT_1 2
#define BUT_2 1

/**
 * Definitions of the pins going to the game leds (white and orange).
 * GAME_LED_PIN[layer][id]
 */
const byte GAME_LED_PIN[2][3] = {
  {10, 8, 7}, {13, 12, 11}
};

/**
 * The [RGB]_MAX_FACTOR should be tuned according to
 * the led's properties in order to get white light
 * with colorize_f(1, 1, 1) or colorize(255, 255, 255).
 */
const float R_MAX_FACTOR = 1,
      G_MAX_FACTOR = 1,
          B_MAX_FACTOR = 1;


/** ---------- Main program's code start ---------- */
// Game's delta time. 
unsigned long dt = 25; //(40 "frames" per second)

/**
 * The first two open leds.
 */
byte noteLedId = 0;
byte nextLedId = 1;

/**
 * Variables for storing the state of the buttons and
 * the state of the bottom layer of the leds (layer 0).
 */
byte ledState = 0;
byte buttonState = 0;

// Initialize the push buttons with their pins.
PushButton button0(BUT_0);
PushButton button1(BUT_1);
PushButton button2(BUT_2);

/**
 * Super Mario Theme
 *  source: https://wiki.mikrotik.com/wiki/Super_Mario_Theme
 * Each triplet contains: frequency, lenght, delay after the note.
 */
unsigned long musicScore[80][3] = {
  {660, 100, 150}, {660, 100, 300}, {660, 100, 300}, {510, 100, 100}, {660, 100, 300}, {770, 100, 550}, {380, 100, 575}, {510, 100, 450}, {380, 100, 400}, {320, 100, 500}, {440, 100, 300}, {480, 80, 330}, {450, 100, 150}, {430, 100, 300}, {380, 100, 200}, {660, 80, 200}, {760, 50, 150}, {860, 100, 300}, {700, 80, 150}, {760, 50, 350}, {660, 80, 300}, {520, 80, 150}, {580, 80, 150}, {480, 80, 500}, {510, 100, 450}, {380, 100, 400}, {320, 100, 500}, {440, 100, 300}, {480, 80, 330}, {450, 100, 150}, {430, 100, 300}, {380, 100, 200}, {660, 80, 200}, {760, 50, 150}, {860, 100, 300}, {700, 80, 150}, {760, 50, 350}, {660, 80, 300}, {520, 80, 150}, {580, 80, 150}, {480, 80, 500}, {500, 100, 300}, {760, 100, 100}, {720, 100, 150}, {680, 100, 150}, {620, 150, 300}, {650, 150, 300}, {380, 100, 150}, {430, 100, 150}, {500, 100, 300}, {430, 100, 150}, {500, 100, 100}, {570, 100, 220}, {500, 100, 300}, {760, 100, 100}, {720, 100, 150}, {680, 100, 150}, {620, 150, 300}, {650, 200, 300}, {1020, 80, 300}, {1020, 80, 150}, {1020, 80, 300}, {380, 100, 300}, {500, 100, 300}, {760, 100, 100}, {720, 100, 150}, {680, 100, 150}, {620, 150, 300}, {650, 150, 300}, {380, 100, 150}, {430, 100, 150}, {500, 100, 300}, {430, 100, 150}, {500, 100, 100}, {570, 100, 420}, {585, 100, 450}, {550, 100, 420}, {500, 100, 360}, {380, 100, 300}, {500, 100, 300},
};
const unsigned long lastNoteId = 80;

/**
 * enum type used for a mini cyclic state machine.
 */
enum ESoundState{
  ESS_NoteReady,
  ESS_NotePlayed,
  ESS_NoteFinished,
};

bool songStarted = false;
bool isGameFinished = false;


double penalty = 0; // Stores the current penalty.
double penaltyStep = 1; // The penalty applied when a wrong button is pressed
            // or there is a lot of delay (check the penaltyTiers)
const unsigned long penaltyTiers[] = {
  500,  // if the button is pressed within 500ms there is no penalty
  1500, // if the button is pressed within 1500ms there is half penalty
  // if the button is pressed after the last tier, there will be full penalty.
};

void setup()
{
  // Game leds
  for (int i = 0; i < 2; i++) 
    for (int j = 0; j < 3; j++)
    pinMode(GAME_LED_PIN[i][j], OUTPUT);

  
  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  
  setUpButtonEvents();
  setupColorPins();
  
  onGameStart();
}

void loop()
{
  // Wait a bit (delta time).
  delay(dt);
  
  if (!isGameFinished) 
  {
    // Check Input.
    button0.Update();
    button1.Update();
    button2.Update();
  // Continue the game loop.
    gameLoop(dt);
  }
}

void gameLoop(unsigned long deltaTime)
{
  static unsigned long currentNote = 0;
  static unsigned long lastTick = 0;
  static unsigned long currentTime = 0;
  static ESoundState soundState = ESS_NoteReady;
  
  if (currentNote == lastNoteId)
  {
    onGameFinished();
    return;
  }
  
  currentTime += dt;
  // Check if the next note is ready and the right button was pressed.
  if (soundState == ESS_NoteReady && (buttonState & ledState))
  {
    if (!songStarted)
    {
      songStarted = true;
      colorize(0, 255, 0);
    }
    else
    {
      long delay = currentTime - lastTick;
      if (delay < penaltyTiers[0])
      {
        colorize(0, 255, 0);
      }
      else if (delay < penaltyTiers[1])
      {
        colorize(255, 255, 0);
        penalty += penaltyStep / 2;
      }
      else
      {
        penalty += penaltyStep;
        colorize(255, 0, 0);
      }
    }
    
    // Get the time.
    lastTick = currentTime;
    
    // Reset button state.
    buttonState = 0;
    
    // Play the note.
    tone(BUZZER_PIN, musicScore[currentNote][0]);
    soundState = ESS_NotePlayed;
    
    // Turn off the leds (resets the led state).
    controlLed(0, noteLedId, false);
    controlLed(1, nextLedId, false);
  }
  // Check if the note's duration has passed.
  else if (soundState == ESS_NotePlayed && 
           currentTime - lastTick >= musicScore[currentNote][1])
  {
    // Stop the note. Start the pause.
    noTone(BUZZER_PIN);
    soundState = ESS_NoteFinished;
    
    // Get the time.
    lastTick = currentTime;
    
    // Prepare the next note.
    noteLedId = nextLedId;
    nextLedId = random(0,3);
    
    if (currentNote != lastNoteId)
      controlLed(0, noteLedId, true);
    
    // Clear color
    colorize(0, 0, 0);
  }
  // Check if the pause's duration has passed.
  else if (soundState == ESS_NoteFinished && 
           currentTime - lastTick >= musicScore[currentNote][2])
  {
    // Current note's pause time is completed. Get to the next one.
    currentNote++;
    soundState = ESS_NoteReady;
    
    if (currentNote != lastNoteId - 1)
      controlLed(1, nextLedId, true);
    
    // Get the time.
    lastTick = currentTime;
  }
}

/**
 * What happens when the game finishes.
 */
void onGameFinished()
{
  isGameFinished = true;
  // Turn off leds.
  for (int i = 0; i < 2; i++) 
    for (int j = 0; j < 3; j++)
    digitalWrite(GAME_LED_PIN[i][j], LOW);
 
  
  // RGB led's color will represent the score.
  // Some blinks to make it more interesting.
  colorize(0, 0, 0);
  delay(900);
  blinkColor(0, 0, 255, 900, 2);
  
  // Result announcement!
  double penaltyFactor = penalty / lastNoteId;
  if (penaltyFactor < 0.2)
    colorize(0, 255, 0);
  else if (penaltyFactor < 0.4)
    colorize(245, 245, 0);
  else
    colorize(255, 0, 0);
}

/**
 * What happens when the game starts.
 */
void onGameStart()
{
  // Turn on the first two leds, one for each layer.
  controlLed(0, noteLedId, true);
  controlLed(1, nextLedId, true);
}

/**
 * Controls the white leds. Sets the led state.
 */
void controlLed(byte layer, byte id, bool turnOn)
{
  digitalWrite(GAME_LED_PIN[layer][id], turnOn);
  
  if (layer == 0)
  {
    ledState = turnOn ? 1 << id : 0;
  }
}

/**
 * Called when the button is pressed (one time).
 */
void onButtonDown(byte buttonId)
{
  buttonState = 1 << buttonId;
  if (!(buttonState & ledState))
  {
    penalty += penaltyStep;
    colorize(255, 0, 0);
  }
}

/**
 * @brief Sets up the buttons' events.
 */
void setUpButtonEvents()
{
  button0.SetOnButtonDownEvent([](void){onButtonDown(0);});
  button1.SetOnButtonDownEvent([](void){onButtonDown(1);});
  button2.SetOnButtonDownEvent([](void){onButtonDown(2);});
}

/**
 * @brief Blinks the specified color @a times.
 */
void blinkColor(byte r, byte g, byte b, long time_ms, byte times)
{
  for (int i = 0; i < times; i++)
  {
    colorize(r, g, b);
    delay(time_ms);
    colorize(0, 0, 0);
    delay(time_ms);
  } 
}

/**
 * @brief Sets up the pins connecting the RGB led to the arduino.
 *
 *   Marks the defined pins as output pins.
 */
void setupColorPins()
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
}

/**
 * @brief Sets the color of the RGB led from [0, 255] values.
 *
 *   Applies [rgb] * 5 * [RGB]_MAX_FACTOR / 255 volt
 *   at the color pins.
 *
 * @param r byte [0, 255] range
 * @param g byte [0, 255] range
 * @param b byte [0, 255] range
 */
void colorize(byte r, byte g, byte b)
{
  analogWrite(RED_PIN, r * R_MAX_FACTOR);
  analogWrite(GREEN_PIN, g * G_MAX_FACTOR);
  analogWrite(BLUE_PIN, b * B_MAX_FACTOR);
}

/** ----- MAIN PROGRAM END ----- */

/**
 * PushButton's function definitions. Normally in a separate file.
 */
PushButton::PushButton(int inputPin)
{
  pin = inputPin;
  isDown = false;
  onButtonDownEvent = 0;
  
  pinMode(pin, INPUT);
}

/**
* @brief Calls the appropriate button events. 
*  
* Should be called inside loop() for the events to work.
* Available events:
*   onButtonDownEvent(): Called when the button is just pressed.
*/
void PushButton::Update()
{
  int input = digitalRead(pin);

  // When the button is just pressed.
  if (input && !isDown)
  {
    isDown = true;
    if (onButtonDownEvent) onButtonDownEvent();
  }
  // When the button is just released.
  else if (!input && isDown)
  {
    isDown = false;
  }
}


void PushButton::SetOnButtonDownEvent(void (*event)(void))
{
  onButtonDownEvent = event;
}
