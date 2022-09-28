/*
Stijn Brugman s2359413 
Jarl M. Sæbø s3105911 
Nils Rutgers s1954377
Gonzalo M. Rodriguez s2331659
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

/* --------- SETTINGS ---------- */
#define NUM_SWEEPS				3
#define NUM_OBJECTS				30
#define OBJECT_MIN_SIZE 		4
#define OBJECT_TRESHOLD 		10

/* -----------	PINS ----------- */
#define TRIGGER_GPIO_PIN		1
#define ECHO_GPIO_PIN			9
#define SERVO_GPIO_PIN			11

/* ------	CONFIG_OFFSETS ----- */
#define ZERO_CMP 				9
#define EN_ALWAYS 				12
#define EN_ONESHOT 				13

/* ----	PWN_BASE_ADDRESSES ----- */
volatile uint32_t *PWM_SERVO 		= (uint32_t*)0x10035000; // 2
volatile uint32_t *PWM_TRIGGER  	= (uint32_t*)0x10015000; // 0


/* ----	GPIO_BASE_ADDRESSES ----- */
volatile uint32_t* GPIO_BASE 		= (uint32_t*)0x10012000;
volatile uint32_t *GPIO_INPUT_EN 	= (uint32_t*)0x10012004;
volatile uint32_t *GPIO_INPUT_VAL 	= (uint32_t*)0x10012000;
volatile uint32_t *GPIO_IOF_EN 		= (uint32_t*)0x10012038;
volatile uint32_t *GPIO_IOF_SEL 	= (uint32_t*)0x1001203C;

/* ----	PWM_COMP_ADDRESSES ------ */
volatile uint32_t *PWM_SERVO_CMP0	= (uint32_t*)0x10035020; //SERVO
volatile uint32_t *PWM_SERVO_CMP1	= (uint32_t*)0x10035024; //SERVO
volatile uint32_t *PWM_TRIGGER_CMP0 = (uint32_t*)0x10015020; //TRIGGER
volatile uint32_t *PWM_TRIGGER_CMP1 = (uint32_t*)0x10015024; //TRIGGER

/* ----	RTC_CONFIG_ADRESSES ----- */
volatile uint32_t *RTC_CONFIG		= (uint32_t*)0x10000040;
volatile uint32_t *RTC_CNT_LOW		= (uint32_t*)0x10000048;
volatile uint32_t *RTC_CNT_HIGH		= (uint32_t*)0x1000004C;

/* ----	RTC_CONFIG_CONSTANTS ----- */
const uint32_t rtc_clock 			= 32768;
const uint64_t micro_multiplier 	= 1000000;

void _delay_ms(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Storing start time
    clock_t start_time = clock();

    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds);
}

void init_PWM_servo(){
	//Clear the entire config register of pwm_servo
	for(int i = 0 ; i < 32; i++){
		*PWM_SERVO &= ~(1<<i);
	}

	//Configure GPIO pin to be a PWM pin
	*GPIO_IOF_EN 	|= (1 << SERVO_GPIO_PIN);
	*GPIO_IOF_SEL 	|= (1 << SERVO_GPIO_PIN);

	//Configure PWM_SERVO config reg
	*PWM_SERVO  = 0b0011;
	*PWM_SERVO |= (1 << EN_ALWAYS);
	*PWM_SERVO |= (1 << ZERO_CMP);

	*PWM_SERVO_CMP0 = (uint16_t)40000;
	*PWM_SERVO_CMP1 = (uint16_t)35000;
}

void init_PWM_trigger(){
	//Clear config register
	for(int i = 0; i < 32; i++){
		*PWM_TRIGGER &= ~(1<<i);
	}

	//Configure GPIO pin to be a PWM pin
	*GPIO_IOF_EN	|= (1 << TRIGGER_GPIO_PIN);
	*GPIO_IOF_SEL	|= (1 << TRIGGER_GPIO_PIN);

	//Configure PWM_TRIGGER config reg
	*PWM_TRIGGER 	 = 0b0001;

	*PWM_TRIGGER_CMP0 = (uint8_t) 1000;
	*PWM_TRIGGER_CMP1 = (uint8_t) 920;
}

uint64_t get_rtc_val_micro(){
	uint32_t rtc_val_low 	= *RTC_CNT_LOW;
	uint32_t rtc_val_high 	= *RTC_CNT_HIGH;
	uint64_t rtc_val = (uint64_t) rtc_val_high << 32 | rtc_val_low;
	return (rtc_val * micro_multiplier / rtc_clock);
}

uint32_t measure_micro_distance(){
	//Set the GPIO pin back to 0
	*GPIO_INPUT_VAL &= ~(1 << ECHO_GPIO_PIN);

	//Send out pulse
	*PWM_TRIGGER 	|= (1 << EN_ONESHOT);

	//wait until first echo pin is received
	while(!((*GPIO_INPUT_VAL >> ECHO_GPIO_PIN) & 0b1));
//	uint64_t before = get_rtc_val_micro();
	clock_t before = clock();

	//wait until echo pin goes to zero
	while(((*GPIO_INPUT_VAL >> ECHO_GPIO_PIN) & 0b1)){
		if (clock() - before > 18000) return 0;
	}

//	uint64_t after = get_rtc_val_micro();
	clock_t after = clock();

	clock_t time_diff = after - before;

	return (uint32_t)(time_diff * 170);
}

uint32_t get_average_distance(uint32_t buffer[], int size){
	if (size == 0) return 0;

	int tmp = 0;
	for(int i = 0; i < size; i++){
		tmp += buffer[i];
	}
	return (uint32_t)(tmp / size);
}


int main() {
	// INIT Distance Sensor, Servo, Internal Timer
	init_PWM_servo();
	*GPIO_INPUT_EN |= (1 << ECHO_GPIO_PIN);
	init_PWM_trigger();
	*RTC_CONFIG |= (1 << EN_ALWAYS);

	uint32_t final_dis[3] = {500, 500, 500};
	uint32_t final_pos[3] = {0};

	int sweeps = 0;
	while(sweeps < NUM_SWEEPS){
		uint32_t objects_dis[NUM_OBJECTS] = {0};
		uint32_t objects_pos[NUM_OBJECTS] = {0};

		uint32_t closest_pos 	= 0;
		uint32_t closest_dis 	= 500; //Larger than max

		int object_size 			= 0;
		int object_index 			= 0;

		// distance buffer to store tmp distances of the measure object
		uint32_t dis_buffer[50] 	= {0};
		int size_dis_buffer 		= 0;

		int j = 0;
		for(j = 0; j < 70; j++){
			*PWM_SERVO_CMP1 = (uint16_t) 35000 + j * 22;
			_delay_ms(40);

			uint32_t distance = 0;

			for(int i = 0; i < 3; i++){
				distance += measure_micro_distance();
			}

			distance = distance / 30000;

			// adding distance to buffer
			dis_buffer[size_dis_buffer] = distance;
			size_dis_buffer++;

			printf("Distance(cm): %d", distance);
			printf(" @ Rotation(Degrees): %d \r\n", j);

			if (abs(get_average_distance(dis_buffer, size_dis_buffer) - distance) > OBJECT_TRESHOLD){
				if (object_size < OBJECT_MIN_SIZE) closest_dis = 500;

				printf("------- NEW OBJECT CREATED ---------\r\n");
				printf("Distance(cm): %d", closest_dis);
				printf(" @ Rotation(Degrees): %d \r\n", closest_pos);
				printf("------------------------------------\r\n");

				// add objects distance & position
				objects_dis[object_index] = closest_dis;
				objects_pos[object_index] = closest_pos;

				// reset closest variables to current distance
				closest_dis = distance;
				closest_pos = j;

				// reset current size of object
				object_size = 1;

				// increment objects array size
				object_index++;

				// Empty tmp buffer and reset counter
				for(int i = 0; i < size_dis_buffer; i++){
					dis_buffer[i] = 0;
				}
				size_dis_buffer = 0;

				//make sure to add the current distance again
				dis_buffer[size_dis_buffer] = distance;
			}


			if(distance < closest_dis){
				closest_dis = distance;
				closest_pos = j;
			}

			object_size++;
		}

		if (object_size > OBJECT_MIN_SIZE){
			objects_dis[object_index] = closest_dis;
			objects_pos[object_index] = j;
		}

		// summarize the detected objects
		for(int i = 0; i < NUM_OBJECTS; i++){
			if (objects_dis[i] == 0 || objects_dis[i] == 500) continue;
			printf("Object %d: ", i);
			printf("Distance(cm): %d", objects_dis[i]);
			printf(" @ Rotation(Degrees): %d \r\n", objects_pos[i]);
		}


		uint32_t tmp_dis[3] = {500, 500, 500};
		uint32_t tmp_pos[3] = {100, 100, 100};

		/* --------------------	Sorting Algorithm ------------------------- */
		for(int i = 0; i < NUM_OBJECTS; i++){
			if(objects_dis[i] < tmp_dis[0] && objects_dis[i] != 0){
				tmp_dis[2] = tmp_dis[1];
				tmp_pos[2] = tmp_pos[1];
				tmp_dis[1] = tmp_dis[0];
				tmp_pos[1] = tmp_pos[0];
				tmp_dis[0] = objects_dis[i];
				tmp_pos[0] = objects_pos[i];
			}
			else if(objects_dis[i] < tmp_dis[1] && objects_dis[i] != 0){
				tmp_dis[2] = tmp_dis[1];
				tmp_pos[2] = tmp_pos[1];
				tmp_dis[1] = objects_dis[i];
				tmp_pos[1] = objects_pos[i];
			}
			else if(objects_dis[i] < tmp_dis[2] && objects_dis[i] != 0){
				tmp_dis[2] = objects_dis[i];
				tmp_pos[2] = objects_pos[i];
			}
		}
		/* ---------------------------------------------------------------- */

		// Print the 3 closest distances
		for(int i =0; i < 3; i++){
			printf("Distance(cm): %d \r\n", tmp_dis[i]);
		}

		// Check if the tmp 'best' distances can be updated in the final
		for(int k = 0; k < 3; k++){
			if(final_dis[k] > tmp_dis[k]){
				final_dis[k] = tmp_dis[k];
				final_pos[k] = tmp_pos[k];
			}
		}

		sweeps++;

		// reset servo to initil position and give it some time
		*PWM_SERVO_CMP1 = (uint16_t) 35000;
		_delay_ms(500);
	}

	// print final objects
	for(int i = 0; i < 3; i++){
		printf("Final distance(cm): %d", final_dis[i]);
		printf(" @ rotation(Degrees): %d \r\n", final_pos[i]);
	}

}
