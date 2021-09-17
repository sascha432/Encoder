/* Encoder Library, for measuring quadrature encoded signals
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 * Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
 *
 * Version 1.2 - fix -2 bug in C-only code
 * Version 1.1 - expand to support boards with up to 60 interrupts
 * Version 1.0 - initial release
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#ifndef Encoder_h_
#define Encoder_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"
#include<helpers.h>//TODO remove

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#    define ENCODER_USE_INTERRUPTS
#    define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#    include "utility/interrupt_pins.h"
#    ifdef ENCODER_OPTIMIZE_INTERRUPTS
#        include "utility/interrupt_config.h"
#    endif
#else
#    define ENCODER_ARGLIST_SIZE 0
#endif

// set to 1, void update(PinStatesType pinStates) must be called to update the state
// after a pin has changed.
// set to 0, the library reads the pins and updates the position automatically
#ifndef ENCODER_USE_DIRECT_PIN_ACCESS
#    define ENCODER_USE_DIRECT_PIN_ACCESS 0
#endif

#if ENCODER_USE_INTERRUPTS && !ENCODER_USE_DIRECT_PIN_ACCESS
#    error ENCODER_USE_INTERRUPTS requires ENCODER_USE_DIRECT_PIN_ACCESS=1
#endif

#ifndef ENCODER_USE_ATOMIC_BLOCK
#    if ENCODER_USE_INTERRUPTS || !ENCODER_USE_DIRECT_PIN_ACCESS
#        define ENCODER_USE_ATOMIC_BLOCK 1
#        if defined(ESP8266) || defined(ESP32)
//TODO
// for eso8266 use esp8266::InterruptLock or ets_intr_lock/unlock
// for esp32, use a semaphore mutex or std::mutex
#            error not supported
#        else
#            include <util/atomic.h>
#            define ENCODER_ATOMIC_BLOCK() ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#		 endif
#    else
#        define ENCODER_USE_ATOMIC_BLOCK 0
#        define ENCODER_ATOMIC_BLOCK()
#    endif
#endif

#if defined(HAVE_UINT24) || defined(ENCODER_POSITION_DATA_TYPE)
#    if HAVE_UINT24 != -1
#        include <int24_types.h>
#    endif
#endif

// Use ICACHE_RAM_ATTR for ISRs to prevent ESP8266 resets
#if defined(ESP8266) || defined(ESP32)
#    define ENCODER_ISR_ATTR ICACHE_RAM_ATTR
#else
#    define ENCODER_ISR_ATTR
#endif

// store additional debug data about acceleration
#ifndef ENCODER_DEBUG
#    define ENCODER_DEBUG 1
#endif

#if ENCODER_DEBUG
	struct diff_t {
		uint32_t millis;
		uint16_t time;
		uint8_t rotations;
		uint32_t acceleration;
		int32_t position;
		diff_t(uint16_t _time, uint8_t _rotations = 0, uint32_t _acceleration = 0, int32_t _position = 0) :
			millis(::millis()),
			time(_time),
			rotations(_rotations),
			acceleration(_acceleration),
			position(_position)
		{}
		diff_t() :
			millis(0),
			time(0),
			rotations(0),
			acceleration(0),
			position(0)
		{}
	};
#endif

// All the data needed by interrupts is consolidated into this ugly struct
// to facilitate assembly language optimizing of the speed critical update.
// The assembly code uses auto-incrementing addressing modes, so the struct
// must remain in exactly this order.
struct Encoder_internal_state_t {
	// these values give fine control in increments of one und up to 64 steps at once with full acceleration
	// a single fast turn can change the value by more than 5000 or just increment it in single steps by turning
	// very slowly. the acceleration is updated every single step and responds very quickly
	//
	// the user can set the amount of acceleration to use

	// reset acceleration if more time (milliseconds) passed between moving the encoder
	static constexpr uint32_t kResetTimeout = 2500;
	// acceleration from 0-255 / 0-100% / basic multiplier
	static constexpr uint8_t kDefaultAcceleration = 128;
	// threshold before activating acceleration
	static constexpr uint32_t kAccelerationThreshold = 300000;
	// shift bits to the right before applying the acceleration value
	// (value - kAccelerationThreshold) >> kAccelerationDivider
	static constexpr uint8_t kAccelerationDivider = 12;
	// max. multiplier per step
	static constexpr uint8_t kAccelerationMaxMultiplier = 64;
	// the weight of new acceleration vs stored in the integration (bit shift, 5 = multiplied by 32)
	// this allow to configure the rate of change independently from the averaging period at no
	// extra cost since it is done in the fixed point alogrihtm
	static constexpr uint8_t kAccelerationWeight = 5;
	// averaging period in milliseconds
	// the movements are integrated into the acceleration value with a rate of 256 units
	// per second and kAccelerationWeight. low values will cause a quick increase and decrease
	// of the acceleration, high values will make it slower to respond
	static constexpr uint32_t kAveragingPeriod = 50;
	// bounce events within 10 milliseconds
	// should be set at least to 2, even with hardware debouncing (which is highly recommended)
	static constexpr uint32_t kDebounceTime = 10;
	// use an average value of the last 5 acceleration values to avoid choppy input
	// the algorithm changes the acceleration each step, but requires turning the
	// encoder very smoothly. the averaging cancels this effect out
	static constexpr uint8_t kAccelerationAverageCount = 5;

	// debug only
	static constexpr uint32_t kInvalidTime = 0x7fffffff;


	#ifdef ENCODER_POSITION_DATA_TYPE
		using position_t = ENCODER_POSITION_DATA_TYPE;
	#else
		using position_t = int32_t;
	#endif

	#if ENCODER_USE_DIRECT_PIN_ACCESS
		volatile IO_REG_TYPE * pin1_register;
		volatile IO_REG_TYPE * pin2_register;
		IO_REG_TYPE            pin1_bitmask;
		IO_REG_TYPE            pin2_bitmask;
	#endif

	position_t position;
	uint32_t last;
	uint32_t currentAcceleration;
	uint32_t averageAcceleration;
	#if ENCODER_DEBUG
		uint32_t time;
		uint16_t duration;
		uint16_t rotations;
		diff_t history[16];
		void clearHistory() {
			memset(history, 0, sizeof(history));
		}
		void addHistory(uint16_t diff) {
			if (!diff) {
				diff = ~0;
			}
			addHistory(diff_t(diff, rotations, currentAcceleration, position));
		}
		void addHistory(const diff_t &data) {
			memmove(&history[1], &history[0], sizeof(history) - sizeof(*history));
			history[0] = data;
		}
		diff_t getHistory() {
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
				auto hPtr = &history[15];
				for(uint8_t i = 0; i < 16; i++, hPtr--) {
					if (hPtr->time) { // find last entry and return it
						diff_t tmp = *hPtr;
						*hPtr = diff_t();
						return tmp;
					}
				}
			}
			return diff_t();
		}
	#endif
	uint8_t state;
	uint8_t acceleration;

	Encoder_internal_state_t() { }

	Encoder_internal_state_t(uint8_t _state) :
		position(0),
		last(0),
		currentAcceleration(0),
		averageAcceleration(0),
		#if ENCODER_DEBUG
			time(kInvalidTime),
			duration(0),
			rotations(0),
			history{},
		#endif
		state(_state),
		acceleration(kDefaultAcceleration)
	{
	}

	// restart acceleration
	void resetDuration() {
		last = millis();
		currentAcceleration = 0;
		averageAcceleration = 0;
		#if ENCODER_DEBUG
			duration = 0;
			rotations = 0;
			time = last;
			clearHistory();
		#endif
	}

	// update duration
	// returns false if the event has bounced
	bool updateDuration() {
		uint32_t time = millis();
		uint32_t diff = time - last;
		if (diff > kResetTimeout) {
			resetDuration();
			#if ENCODER_DEBUG
				addHistory(diff_t(diff, 0, 0, 99999));
			#endif
			return true;
		}
		if (diff < kDebounceTime) {
			#if ENCODER_DEBUG
				addHistory(diff_t(diff, rotations, currentAcceleration, 77777));
			#endif
			return false;
		}
		last = time;
		#if ENCODER_DEBUG
			if (this->time == kInvalidTime) {
				this->time = time;
			}
			uint32_t tmp = time - this->time;
			duration = (tmp < kInvalidTime) ? tmp : kInvalidTime;
		#endif
		auto diff2 = diff;
		if (diff2 < kDebounceTime) {
			diff2 = kDebounceTime;
		}
		// the multiplier adds existing acceleration over kAveragingPeriod
		uint32_t multiplier = kAveragingPeriod / diff2;
		// add 256 acceleration points per second
		// 1000 * 256 / diff
		uint32_t newValue = ((1000UL << (8/* 256 */ + kAccelerationWeight)) / diff2);
		// integrate new acceleration
		currentAcceleration = ((currentAcceleration * multiplier) + newValue) / (multiplier + 1);
		#if ENCODER_DEBUG
			addHistory(diff);
		#endif
		return true;
	}

	// add value to average and return average
	uint32_t addToAverage(uint32_t value) {
		averageAcceleration = ((averageAcceleration * (kAccelerationAverageCount - 1)) + value) / kAccelerationAverageCount;
		return averageAcceleration;
	}

	int16_t getSteps(int8_t steps) {
		auto result = _getSteps(steps);
		#if ENCODER_DEBUG && 0
			NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
				Serial.printf_P(PSTR("%d->%d (%ld)\n"), steps, result, averageAcceleration);
			}
		#endif
		return result;
	}

	// get steps with acceleration
	int16_t _getSteps(int8_t steps)
	{
		if (!steps || !acceleration || currentAcceleration <= kAccelerationThreshold) {
			addToAverage(0);
			return steps;
		}
		// get up to 100% of currentAcceleration, depending on the user settings
		auto tmp = getAcceleration(currentAcceleration - kAccelerationThreshold);
		// multiply the acceleration factor with the steps
		int32_t retval = steps * addToAverage(tmp);
		retval >>= kAccelerationDivider;

		int16_t limit = steps * kAccelerationMaxMultiplier;
		if (steps < 0) {
			// return value must be smaller or equal steps and above limit
			if (retval <= limit) {
				return limit;
			}
			if (retval < steps) {
				return retval;
			}
			return steps;
		}
		else {
			// return value must be greater or equal steps and below limit
			if (retval >= limit) {
				return limit;
			}
			if (retval > steps) {
				return retval;
			}
			return steps;
		}
	}

	uint32_t getAcceleration(uint32_t accelerationValue) const {
		return ((accelerationValue) * (acceleration + 1)) >> 8;
	}

	void setAcceleration(uint8_t _acceleration) {
		acceleration = _acceleration;
	}

	#if ENCODER_DEBUG
		void dump(Stream &output) {
			output.printf_P(PSTR("last=%ld ac=%ld dur=%u rot=%u spd=%u pos=%ld\n"),
				(long)last,
				(long)currentAcceleration,
				duration,
				rotations,
				/*getSteps(1)*/0,
				(long)position
			);
			output.flush();

			dumpHistory(output);
		}

		void dumpHistory(Stream &output) {
			auto h = getHistory();
			while (h.time) {
				char buf[32];
				PrintBuffer pb(buf, sizeof(buf));
				pb.print(getAcceleration(std::max<int32_t>(0, h.acceleration - kAccelerationThreshold)) / (float)(1UL << (kAccelerationWeight + kAccelerationDivider)), 2);
				output.printf_P(PSTR("H t=%07ld T=%05u p=%05ld ai=%-11.11s"),
					(long)h.millis,
					(unsigned)h.time,
					(long)h.position,
					buf
				);
				output.flush();
				output.printf_P(PSTR(" a=%ld / %ld\n"),
					(long)getAcceleration(h.acceleration),
					(long)h.acceleration
				);
				output.flush();
				h = getHistory();
			}
		}
	#endif

};

class Encoder
{
public:
	Encoder(uint8_t pin1, uint8_t pin2) {
		#ifdef INPUT_PULLUP
			pinMode(pin1, INPUT_PULLUP);
			pinMode(pin2, INPUT_PULLUP);
		#else
			pinMode(pin1, INPUT);
			digitalWrite(pin1, HIGH);
			pinMode(pin2, INPUT);
			digitalWrite(pin2, HIGH);
		#endif
		#if ENCODER_USE_DIRECT_PIN_ACCESS
			encoder.pin1_register = PIN_TO_BASEREG(pin1);
			encoder.pin1_bitmask = PIN_TO_BITMASK(pin1);
			encoder.pin2_register = PIN_TO_BASEREG(pin2);
			encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
		#endif
		// allow time for a passive R-C filter to charge
		// through the pullup resistors, before reading
		// the initial state
		delayMicroseconds(2000);
		uint8_t s = 0;
		#if ENCODER_USE_DIRECT_PIN_ACCESS
			if (DIRECT_PIN_READ(encoder.pin1_register, encoder.pin1_bitmask)) s = static_cast<uint8_t>(PinStatesType::P1_HIGH);
			if (DIRECT_PIN_READ(encoder.pin2_register, encoder.pin2_bitmask)) s |= static_cast<uint8_t>(PinStatesType::P2_HIGH);
		#else
			if (digitalRead(pin1)) s = static_cast<uint8_t>(PinStatesType::P1_HIGH);
			if (digitalRead(pin2)) s |= static_cast<uint8_t>(PinStatesType::P2_HIGH);
		#endif
		encoder = Encoder_internal_state_t(0);
		__updateState(static_cast<PinStatesType>(s));
		#ifdef ENCODER_USE_INTERRUPTS
			interrupts_in_use = attach_interrupt(pin1, &encoder);
			interrupts_in_use += attach_interrupt(pin2, &encoder);
		#endif
	}

	inline Encoder_internal_state_t::position_t read() {
		ENCODER_ATOMIC_BLOCK() {
			#ifdef ENCODER_USE_INTERRUPTS
				if (interrupts_in_use < 2) {
					update(&encoder);
				}
			#elif ENCODER_USE_DIRECT_PIN_ACCESS
				update(&encoder);
			#elif !ENCODER_USE_DIRECT_PIN_ACCESS
				// skip update
			#else
				#error invalid configuration
			#endif
			return encoder.position;
		}
		__builtin_unreachable();
	}

	inline Encoder_internal_state_t::position_t readAndReset() {
		ENCODER_ATOMIC_BLOCK() {
			#ifdef ENCODER_USE_INTERRUPTS
				if (interrupts_in_use < 2) {
					update(&encoder);
				}
			#elif ENCODER_USE_DIRECT_PIN_ACCESS
				update(&encoder);
			#elif !ENCODER_USE_DIRECT_PIN_ACCESS
				// skip update
			#else
				#error invalid configuration
			#endif
			auto ret = encoder.position;
			encoder.position = 0;
			return ret;
		}
		__builtin_unreachable();
	}

	inline void write(Encoder_internal_state_t::position_t p) {
		ENCODER_ATOMIC_BLOCK() {
			encoder.position = p;
			encoder.resetDuration();
		}
	}

	// same as write but does not reset the acceleration
	inline void reset(Encoder_internal_state_t::position_t p) {
		ENCODER_ATOMIC_BLOCK() {
			encoder.position = p;
		}
	}

 	// bits for the pin states
	enum class PinStatesType : uint8_t {
		NONE = 0,
		P1_HIGH = _BV(0),
		P2_HIGH = _BV(1),
		// P1_HIGH = _BV(2), // for old algorihtm
		// P2_HIGH = _BV(3),
		P1_P2_HIGH = P1_HIGH | P2_HIGH
	};

	int8_t __updateState(PinStatesType pinStates);

	// update encoder by passing the states of the encoder pins
	// interrupts must be locked if not called from inside ISR
	inline void update(PinStatesType pinStates)	{
		#if 0
			// old algorithm that has some issues
			int8_t steps;
			uint8_t state = (encoder.state & 3) | static_cast<uint8_t>(pinStates);
			encoder.state = state >> 2;
			switch (state) {
				case 0:
				case 5:
				case 10:
				case 15:
					return;
				case 1:
				case 7:
				case 8:
				case 14:
					steps = 1;
					break;
				case 2:
				case 4:
				case 11:
				case 13:
					steps = -1;
					break;
				case 3:
				case 12:
					steps = 2;
					break;
				default:
					steps = -2;
					break;
			}
			if (encoder.updateDuration()) {
				encoder.position += encoder.getSteps(steps);
				#if ENCODER_DEBUG
					encoder.rotations += (steps >= 0) ? steps : -steps;
				#endif
			}
		#else
			auto steps = __updateState(pinStates);
			encoder.updateDuration();
			if (steps) {
				encoder.position += encoder.getSteps(steps);
				#if ENCODER_DEBUG
					encoder.rotations += (steps >= 0) ? steps : -steps;
				#endif
			}
		#endif
	}

#if !ENCODER_DEBUG
private:
#endif
	Encoder_internal_state_t encoder;
#ifdef ENCODER_USE_INTERRUPTS
	uint8_t interrupts_in_use;
#endif
public:
#if ENCODER_ARGLIST_SIZE
	static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];
#endif

//                           _______         _______
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	0	0	0	no movement
		//	0	0	0	1	+1
		//	0	0	1	0	-1
		//	0	0	1	1	+2  (assume pin1 edges only)
		//	0	1	0	0	-1
		//	0	1	0	1	no movement
		//	0	1	1	0	-2  (assume pin1 edges only)
		//	0	1	1	1	+1
		//	1	0	0	0	+1
		//	1	0	0	1	-2  (assume pin1 edges only)
		//	1	0	1	0	no movement
		//	1	0	1	1	-1
		//	1	1	0	0	+2  (assume pin1 edges only)
		//	1	1	0	1	-1
		//	1	1	1	0	+1
		//	1	1	1	1	no movement
/*
	// Simple, easy-to-read "documentation" version :-)
	//
	void update(void) {
		uint8_t s = state & 3;
		if (digitalRead(pin1)) s |= 4;
		if (digitalRead(pin2)) s |= 8;
		switch (s) {
			case 0: case 5: case 10: case 15:
				break;
			case 1: case 7: case 8: case 14:
				position++; break;
			case 2: case 4: case 11: case 13:
				position--; break;
			case 3: case 12:
				position += 2; break;
			default:
				position -= 2; break;
		}
		state = (s >> 2);
	}
*/

public:
#if ENCODER_USE_DIRECT_PIN_ACCESS
	// update() is not meant to be called from outside Encoder,
	// but it is public to allow static interrupt routines.
	// DO NOT call update() directly from sketches.
	static void ENCODER_ISR_ATTR update(Encoder_internal_state_t *arg) {
		// read pins and pass value to update method
		PinStatesType val = DIRECT_PIN_READ(arg->pin1_register, arg->pin1_bitmask) ? PinStatesType::P1_HIGH : PinStatesType::NONE;
		if (DIRECT_PIN_READ(arg->pin2_register, arg->pin2_bitmask)) {
			val = static_cast<PinStatesType>(static_cast<uint8_t>(val) | static_cast<uint8_t>(PinStatesType::P2_HIGH));
		}
		update(val);
	}
#endif

#ifdef ENCODER_USE_INTERRUPTS
	// this giant function is an unfortunate consequence of Arduino's
	// attachInterrupt function not supporting any way to pass a pointer
	// or other context to the attached function.
	static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state) {
		switch (pin) {
		#ifdef CORE_INT0_PIN
			case CORE_INT0_PIN:
				interruptArgs[0] = state;
				attachInterrupt(0, isr0, CHANGE);
				break;
		#endif
		#ifdef CORE_INT1_PIN
			case CORE_INT1_PIN:
				interruptArgs[1] = state;
				attachInterrupt(1, isr1, CHANGE);
				break;
		#endif
		#ifdef CORE_INT2_PIN
			case CORE_INT2_PIN:
				interruptArgs[2] = state;
				attachInterrupt(2, isr2, CHANGE);
				break;
		#endif
		#ifdef CORE_INT3_PIN
			case CORE_INT3_PIN:
				interruptArgs[3] = state;
				attachInterrupt(3, isr3, CHANGE);
				break;
		#endif
		#ifdef CORE_INT4_PIN
			case CORE_INT4_PIN:
				interruptArgs[4] = state;
				attachInterrupt(4, isr4, CHANGE);
				break;
		#endif
		#ifdef CORE_INT5_PIN
			case CORE_INT5_PIN:
				interruptArgs[5] = state;
				attachInterrupt(5, isr5, CHANGE);
				break;
		#endif
		#ifdef CORE_INT6_PIN
			case CORE_INT6_PIN:
				interruptArgs[6] = state;
				attachInterrupt(6, isr6, CHANGE);
				break;
		#endif
		#ifdef CORE_INT7_PIN
			case CORE_INT7_PIN:
				interruptArgs[7] = state;
				attachInterrupt(7, isr7, CHANGE);
				break;
		#endif
		#ifdef CORE_INT8_PIN
			case CORE_INT8_PIN:
				interruptArgs[8] = state;
				attachInterrupt(8, isr8, CHANGE);
				break;
		#endif
		#ifdef CORE_INT9_PIN
			case CORE_INT9_PIN:
				interruptArgs[9] = state;
				attachInterrupt(9, isr9, CHANGE);
				break;
		#endif
		#ifdef CORE_INT10_PIN
			case CORE_INT10_PIN:
				interruptArgs[10] = state;
				attachInterrupt(10, isr10, CHANGE);
				break;
		#endif
		#ifdef CORE_INT11_PIN
			case CORE_INT11_PIN:
				interruptArgs[11] = state;
				attachInterrupt(11, isr11, CHANGE);
				break;
		#endif
		#ifdef CORE_INT12_PIN
			case CORE_INT12_PIN:
				interruptArgs[12] = state;
				attachInterrupt(12, isr12, CHANGE);
				break;
		#endif
		#ifdef CORE_INT13_PIN
			case CORE_INT13_PIN:
				interruptArgs[13] = state;
				attachInterrupt(13, isr13, CHANGE);
				break;
		#endif
		#ifdef CORE_INT14_PIN
			case CORE_INT14_PIN:
				interruptArgs[14] = state;
				attachInterrupt(14, isr14, CHANGE);
				break;
		#endif
		#ifdef CORE_INT15_PIN
			case CORE_INT15_PIN:
				interruptArgs[15] = state;
				attachInterrupt(15, isr15, CHANGE);
				break;
		#endif
		#ifdef CORE_INT16_PIN
			case CORE_INT16_PIN:
				interruptArgs[16] = state;
				attachInterrupt(16, isr16, CHANGE);
				break;
		#endif
		#ifdef CORE_INT17_PIN
			case CORE_INT17_PIN:
				interruptArgs[17] = state;
				attachInterrupt(17, isr17, CHANGE);
				break;
		#endif
		#ifdef CORE_INT18_PIN
			case CORE_INT18_PIN:
				interruptArgs[18] = state;
				attachInterrupt(18, isr18, CHANGE);
				break;
		#endif
		#ifdef CORE_INT19_PIN
			case CORE_INT19_PIN:
				interruptArgs[19] = state;
				attachInterrupt(19, isr19, CHANGE);
				break;
		#endif
		#ifdef CORE_INT20_PIN
			case CORE_INT20_PIN:
				interruptArgs[20] = state;
				attachInterrupt(20, isr20, CHANGE);
				break;
		#endif
		#ifdef CORE_INT21_PIN
			case CORE_INT21_PIN:
				interruptArgs[21] = state;
				attachInterrupt(21, isr21, CHANGE);
				break;
		#endif
		#ifdef CORE_INT22_PIN
			case CORE_INT22_PIN:
				interruptArgs[22] = state;
				attachInterrupt(22, isr22, CHANGE);
				break;
		#endif
		#ifdef CORE_INT23_PIN
			case CORE_INT23_PIN:
				interruptArgs[23] = state;
				attachInterrupt(23, isr23, CHANGE);
				break;
		#endif
		#ifdef CORE_INT24_PIN
			case CORE_INT24_PIN:
				interruptArgs[24] = state;
				attachInterrupt(24, isr24, CHANGE);
				break;
		#endif
		#ifdef CORE_INT25_PIN
			case CORE_INT25_PIN:
				interruptArgs[25] = state;
				attachInterrupt(25, isr25, CHANGE);
				break;
		#endif
		#ifdef CORE_INT26_PIN
			case CORE_INT26_PIN:
				interruptArgs[26] = state;
				attachInterrupt(26, isr26, CHANGE);
				break;
		#endif
		#ifdef CORE_INT27_PIN
			case CORE_INT27_PIN:
				interruptArgs[27] = state;
				attachInterrupt(27, isr27, CHANGE);
				break;
		#endif
		#ifdef CORE_INT28_PIN
			case CORE_INT28_PIN:
				interruptArgs[28] = state;
				attachInterrupt(28, isr28, CHANGE);
				break;
		#endif
		#ifdef CORE_INT29_PIN
			case CORE_INT29_PIN:
				interruptArgs[29] = state;
				attachInterrupt(29, isr29, CHANGE);
				break;
		#endif

		#ifdef CORE_INT30_PIN
			case CORE_INT30_PIN:
				interruptArgs[30] = state;
				attachInterrupt(30, isr30, CHANGE);
				break;
		#endif
		#ifdef CORE_INT31_PIN
			case CORE_INT31_PIN:
				interruptArgs[31] = state;
				attachInterrupt(31, isr31, CHANGE);
				break;
		#endif
		#ifdef CORE_INT32_PIN
			case CORE_INT32_PIN:
				interruptArgs[32] = state;
				attachInterrupt(32, isr32, CHANGE);
				break;
		#endif
		#ifdef CORE_INT33_PIN
			case CORE_INT33_PIN:
				interruptArgs[33] = state;
				attachInterrupt(33, isr33, CHANGE);
				break;
		#endif
		#ifdef CORE_INT34_PIN
			case CORE_INT34_PIN:
				interruptArgs[34] = state;
				attachInterrupt(34, isr34, CHANGE);
				break;
		#endif
		#ifdef CORE_INT35_PIN
			case CORE_INT35_PIN:
				interruptArgs[35] = state;
				attachInterrupt(35, isr35, CHANGE);
				break;
		#endif
		#ifdef CORE_INT36_PIN
			case CORE_INT36_PIN:
				interruptArgs[36] = state;
				attachInterrupt(36, isr36, CHANGE);
				break;
		#endif
		#ifdef CORE_INT37_PIN
			case CORE_INT37_PIN:
				interruptArgs[37] = state;
				attachInterrupt(37, isr37, CHANGE);
				break;
		#endif
		#ifdef CORE_INT38_PIN
			case CORE_INT38_PIN:
				interruptArgs[38] = state;
				attachInterrupt(38, isr38, CHANGE);
				break;
		#endif
		#ifdef CORE_INT39_PIN
			case CORE_INT39_PIN:
				interruptArgs[39] = state;
				attachInterrupt(39, isr39, CHANGE);
				break;
		#endif
		#ifdef CORE_INT40_PIN
			case CORE_INT40_PIN:
				interruptArgs[40] = state;
				attachInterrupt(40, isr40, CHANGE);
				break;
		#endif
		#ifdef CORE_INT41_PIN
			case CORE_INT41_PIN:
				interruptArgs[41] = state;
				attachInterrupt(41, isr41, CHANGE);
				break;
		#endif
		#ifdef CORE_INT42_PIN
			case CORE_INT42_PIN:
				interruptArgs[42] = state;
				attachInterrupt(42, isr42, CHANGE);
				break;
		#endif
		#ifdef CORE_INT43_PIN
			case CORE_INT43_PIN:
				interruptArgs[43] = state;
				attachInterrupt(43, isr43, CHANGE);
				break;
		#endif
		#ifdef CORE_INT44_PIN
			case CORE_INT44_PIN:
				interruptArgs[44] = state;
				attachInterrupt(44, isr44, CHANGE);
				break;
		#endif
		#ifdef CORE_INT45_PIN
			case CORE_INT45_PIN:
				interruptArgs[45] = state;
				attachInterrupt(45, isr45, CHANGE);
				break;
		#endif
		#ifdef CORE_INT46_PIN
			case CORE_INT46_PIN:
				interruptArgs[46] = state;
				attachInterrupt(46, isr46, CHANGE);
				break;
		#endif
		#ifdef CORE_INT47_PIN
			case CORE_INT47_PIN:
				interruptArgs[47] = state;
				attachInterrupt(47, isr47, CHANGE);
				break;
		#endif
		#ifdef CORE_INT48_PIN
			case CORE_INT48_PIN:
				interruptArgs[48] = state;
				attachInterrupt(48, isr48, CHANGE);
				break;
		#endif
		#ifdef CORE_INT49_PIN
			case CORE_INT49_PIN:
				interruptArgs[49] = state;
				attachInterrupt(49, isr49, CHANGE);
				break;
		#endif
		#ifdef CORE_INT50_PIN
			case CORE_INT50_PIN:
				interruptArgs[50] = state;
				attachInterrupt(50, isr50, CHANGE);
				break;
		#endif
		#ifdef CORE_INT51_PIN
			case CORE_INT51_PIN:
				interruptArgs[51] = state;
				attachInterrupt(51, isr51, CHANGE);
				break;
		#endif
		#ifdef CORE_INT52_PIN
			case CORE_INT52_PIN:
				interruptArgs[52] = state;
				attachInterrupt(52, isr52, CHANGE);
				break;
		#endif
		#ifdef CORE_INT53_PIN
			case CORE_INT53_PIN:
				interruptArgs[53] = state;
				attachInterrupt(53, isr53, CHANGE);
				break;
		#endif
		#ifdef CORE_INT54_PIN
			case CORE_INT54_PIN:
				interruptArgs[54] = state;
				attachInterrupt(54, isr54, CHANGE);
				break;
		#endif
		#ifdef CORE_INT55_PIN
			case CORE_INT55_PIN:
				interruptArgs[55] = state;
				attachInterrupt(55, isr55, CHANGE);
				break;
		#endif
		#ifdef CORE_INT56_PIN
			case CORE_INT56_PIN:
				interruptArgs[56] = state;
				attachInterrupt(56, isr56, CHANGE);
				break;
		#endif
		#ifdef CORE_INT57_PIN
			case CORE_INT57_PIN:
				interruptArgs[57] = state;
				attachInterrupt(57, isr57, CHANGE);
				break;
		#endif
		#ifdef CORE_INT58_PIN
			case CORE_INT58_PIN:
				interruptArgs[58] = state;
				attachInterrupt(58, isr58, CHANGE);
				break;
		#endif
		#ifdef CORE_INT59_PIN
			case CORE_INT59_PIN:
				interruptArgs[59] = state;
				attachInterrupt(59, isr59, CHANGE);
				break;
		#endif
			default:
				return 0;
		}
		return 1;
	}
#endif // ENCODER_USE_INTERRUPTS


#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)
	#ifdef CORE_INT0_PIN
	static ENCODER_ISR_ATTR void isr0(void) { update(interruptArgs[0]); }
	#endif
	#ifdef CORE_INT1_PIN
	static ENCODER_ISR_ATTR void isr1(void) { update(interruptArgs[1]); }
	#endif
	#ifdef CORE_INT2_PIN
	static ENCODER_ISR_ATTR void isr2(void) { update(interruptArgs[2]); }
	#endif
	#ifdef CORE_INT3_PIN
	static ENCODER_ISR_ATTR void isr3(void) { update(interruptArgs[3]); }
	#endif
	#ifdef CORE_INT4_PIN
	static ENCODER_ISR_ATTR void isr4(void) { update(interruptArgs[4]); }
	#endif
	#ifdef CORE_INT5_PIN
	static ENCODER_ISR_ATTR void isr5(void) { update(interruptArgs[5]); }
	#endif
	#ifdef CORE_INT6_PIN
	static ENCODER_ISR_ATTR void isr6(void) { update(interruptArgs[6]); }
	#endif
	#ifdef CORE_INT7_PIN
	static ENCODER_ISR_ATTR void isr7(void) { update(interruptArgs[7]); }
	#endif
	#ifdef CORE_INT8_PIN
	static ENCODER_ISR_ATTR void isr8(void) { update(interruptArgs[8]); }
	#endif
	#ifdef CORE_INT9_PIN
	static ENCODER_ISR_ATTR void isr9(void) { update(interruptArgs[9]); }
	#endif
	#ifdef CORE_INT10_PIN
	static ENCODER_ISR_ATTR void isr10(void) { update(interruptArgs[10]); }
	#endif
	#ifdef CORE_INT11_PIN
	static ENCODER_ISR_ATTR void isr11(void) { update(interruptArgs[11]); }
	#endif
	#ifdef CORE_INT12_PIN
	static ENCODER_ISR_ATTR void isr12(void) { update(interruptArgs[12]); }
	#endif
	#ifdef CORE_INT13_PIN
	static ENCODER_ISR_ATTR void isr13(void) { update(interruptArgs[13]); }
	#endif
	#ifdef CORE_INT14_PIN
	static ENCODER_ISR_ATTR void isr14(void) { update(interruptArgs[14]); }
	#endif
	#ifdef CORE_INT15_PIN
	static ENCODER_ISR_ATTR void isr15(void) { update(interruptArgs[15]); }
	#endif
	#ifdef CORE_INT16_PIN
	static ENCODER_ISR_ATTR void isr16(void) { update(interruptArgs[16]); }
	#endif
	#ifdef CORE_INT17_PIN
	static ENCODER_ISR_ATTR void isr17(void) { update(interruptArgs[17]); }
	#endif
	#ifdef CORE_INT18_PIN
	static ENCODER_ISR_ATTR void isr18(void) { update(interruptArgs[18]); }
	#endif
	#ifdef CORE_INT19_PIN
	static ENCODER_ISR_ATTR void isr19(void) { update(interruptArgs[19]); }
	#endif
	#ifdef CORE_INT20_PIN
	static ENCODER_ISR_ATTR void isr20(void) { update(interruptArgs[20]); }
	#endif
	#ifdef CORE_INT21_PIN
	static ENCODER_ISR_ATTR void isr21(void) { update(interruptArgs[21]); }
	#endif
	#ifdef CORE_INT22_PIN
	static ENCODER_ISR_ATTR void isr22(void) { update(interruptArgs[22]); }
	#endif
	#ifdef CORE_INT23_PIN
	static ENCODER_ISR_ATTR void isr23(void) { update(interruptArgs[23]); }
	#endif
	#ifdef CORE_INT24_PIN
	static ENCODER_ISR_ATTR void isr24(void) { update(interruptArgs[24]); }
	#endif
	#ifdef CORE_INT25_PIN
	static ENCODER_ISR_ATTR void isr25(void) { update(interruptArgs[25]); }
	#endif
	#ifdef CORE_INT26_PIN
	static ENCODER_ISR_ATTR void isr26(void) { update(interruptArgs[26]); }
	#endif
	#ifdef CORE_INT27_PIN
	static ENCODER_ISR_ATTR void isr27(void) { update(interruptArgs[27]); }
	#endif
	#ifdef CORE_INT28_PIN
	static ENCODER_ISR_ATTR void isr28(void) { update(interruptArgs[28]); }
	#endif
	#ifdef CORE_INT29_PIN
	static ENCODER_ISR_ATTR void isr29(void) { update(interruptArgs[29]); }
	#endif
	#ifdef CORE_INT30_PIN
	static ENCODER_ISR_ATTR void isr30(void) { update(interruptArgs[30]); }
	#endif
	#ifdef CORE_INT31_PIN
	static ENCODER_ISR_ATTR void isr31(void) { update(interruptArgs[31]); }
	#endif
	#ifdef CORE_INT32_PIN
	static ENCODER_ISR_ATTR void isr32(void) { update(interruptArgs[32]); }
	#endif
	#ifdef CORE_INT33_PIN
	static ENCODER_ISR_ATTR void isr33(void) { update(interruptArgs[33]); }
	#endif
	#ifdef CORE_INT34_PIN
	static ENCODER_ISR_ATTR void isr34(void) { update(interruptArgs[34]); }
	#endif
	#ifdef CORE_INT35_PIN
	static ENCODER_ISR_ATTR void isr35(void) { update(interruptArgs[35]); }
	#endif
	#ifdef CORE_INT36_PIN
	static ENCODER_ISR_ATTR void isr36(void) { update(interruptArgs[36]); }
	#endif
	#ifdef CORE_INT37_PIN
	static ENCODER_ISR_ATTR void isr37(void) { update(interruptArgs[37]); }
	#endif
	#ifdef CORE_INT38_PIN
	static ENCODER_ISR_ATTR void isr38(void) { update(interruptArgs[38]); }
	#endif
	#ifdef CORE_INT39_PIN
	static ENCODER_ISR_ATTR void isr39(void) { update(interruptArgs[39]); }
	#endif
	#ifdef CORE_INT40_PIN
	static ENCODER_ISR_ATTR void isr40(void) { update(interruptArgs[40]); }
	#endif
	#ifdef CORE_INT41_PIN
	static ENCODER_ISR_ATTR void isr41(void) { update(interruptArgs[41]); }
	#endif
	#ifdef CORE_INT42_PIN
	static ENCODER_ISR_ATTR void isr42(void) { update(interruptArgs[42]); }
	#endif
	#ifdef CORE_INT43_PIN
	static ENCODER_ISR_ATTR void isr43(void) { update(interruptArgs[43]); }
	#endif
	#ifdef CORE_INT44_PIN
	static ENCODER_ISR_ATTR void isr44(void) { update(interruptArgs[44]); }
	#endif
	#ifdef CORE_INT45_PIN
	static ENCODER_ISR_ATTR void isr45(void) { update(interruptArgs[45]); }
	#endif
	#ifdef CORE_INT46_PIN
	static ENCODER_ISR_ATTR void isr46(void) { update(interruptArgs[46]); }
	#endif
	#ifdef CORE_INT47_PIN
	static ENCODER_ISR_ATTR void isr47(void) { update(interruptArgs[47]); }
	#endif
	#ifdef CORE_INT48_PIN
	static ENCODER_ISR_ATTR void isr48(void) { update(interruptArgs[48]); }
	#endif
	#ifdef CORE_INT49_PIN
	static ENCODER_ISR_ATTR void isr49(void) { update(interruptArgs[49]); }
	#endif
	#ifdef CORE_INT50_PIN
	static ENCODER_ISR_ATTR void isr50(void) { update(interruptArgs[50]); }
	#endif
	#ifdef CORE_INT51_PIN
	static ENCODER_ISR_ATTR void isr51(void) { update(interruptArgs[51]); }
	#endif
	#ifdef CORE_INT52_PIN
	static ENCODER_ISR_ATTR void isr52(void) { update(interruptArgs[52]); }
	#endif
	#ifdef CORE_INT53_PIN
	static ENCODER_ISR_ATTR void isr53(void) { update(interruptArgs[53]); }
	#endif
	#ifdef CORE_INT54_PIN
	static ENCODER_ISR_ATTR void isr54(void) { update(interruptArgs[54]); }
	#endif
	#ifdef CORE_INT55_PIN
	static ENCODER_ISR_ATTR void isr55(void) { update(interruptArgs[55]); }
	#endif
	#ifdef CORE_INT56_PIN
	static ENCODER_ISR_ATTR void isr56(void) { update(interruptArgs[56]); }
	#endif
	#ifdef CORE_INT57_PIN
	static ENCODER_ISR_ATTR void isr57(void) { update(interruptArgs[57]); }
	#endif
	#ifdef CORE_INT58_PIN
	static ENCODER_ISR_ATTR void isr58(void) { update(interruptArgs[58]); }
	#endif
	#ifdef CORE_INT59_PIN
	static ENCODER_ISR_ATTR void isr59(void) { update(interruptArgs[59]); }
	#endif
#endif
};

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
#if defined(INT2_vect) && CORE_NUM_INTERRUPT > 2
ISR(INT2_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(2)]); }
#endif
#if defined(INT3_vect) && CORE_NUM_INTERRUPT > 3
ISR(INT3_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(3)]); }
#endif
#if defined(INT4_vect) && CORE_NUM_INTERRUPT > 4
ISR(INT4_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(4)]); }
#endif
#if defined(INT5_vect) && CORE_NUM_INTERRUPT > 5
ISR(INT5_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(5)]); }
#endif
#if defined(INT6_vect) && CORE_NUM_INTERRUPT > 6
ISR(INT6_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(6)]); }
#endif
#if defined(INT7_vect) && CORE_NUM_INTERRUPT > 7
ISR(INT7_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(7)]); }
#endif
#endif // AVR
#if defined(attachInterrupt)
// Don't intefere with other libraries or sketch use of attachInterrupt()
// https://github.com/PaulStoffregen/Encoder/issues/8
#undef attachInterrupt
#endif
#endif // ENCODER_OPTIMIZE_INTERRUPTS



// https://github.com/buxtronix/arduino/blob/master/libraries/Rotary/Rotary.cpp

/* Rotary encoder handler for arduino. v1.1
 *
 * Copyright 2011 Ben Buxton. Licenced under the GNU GPL Version 3.
 * Contact: bb@cactii.net
 *
 * A typical mechanical rotary encoder emits a two bit gray code
 * on 3 output pins. Every step in the output (often accompanied
 * by a physical 'click') generates a specific sequence of output
 * codes on the pins.
 *
 * There are 3 pins used for the rotary encoding - one common and
 * two 'bit' pins.
 *
 * The following is the typical sequence of code on the output when
 * moving from one step to the next:
 *
 *   Position   Bit1   Bit2
 *   ----------------------
 *     Step1     0      0
 *      1/4      1      0
 *      1/2      1      1
 *      3/4      0      1
 *     Step2     0      0
 *
 * From this table, we can see that when moving from one 'click' to
 * the next, there are 4 changes in the output code.
 *
 * - From an initial 0 - 0, Bit1 goes high, Bit0 stays low.
 * - Then both bits are high, halfway through the step.
 * - Then Bit1 goes low, but Bit2 stays high.
 * - Finally at the end of the step, both bits return to 0.
 *
 * Detecting the direction is easy - the table simply goes in the other
 * direction (read up instead of down).
 *
 * To decode this, we use a simple state machine. Every time the output
 * code changes, it follows state, until finally a full steps worth of
 * code is received (in the correct order). At the final 0-0, it returns
 * a value indicating a step in one direction or the other.
 *
 * It's also possible to use 'half-step' mode. This just emits an event
 * at both the 0-0 and 1-1 positions. This might be useful for some
 * encoders where you want to detect all positions.
 *
 * If an invalid state happens (for example we go from '0-1' straight
 * to '1-0'), the state machine resets to the start until 0-0 and the
 * next valid codes occur.
 *
 * The biggest advantage of using a state machine over other algorithms
 * is that this has inherent debounce built in. Other algorithms emit spurious
 * output with switch bounce, but this one will simply flip between
 * sub-states until the bounce settles, then continue along the state
 * machine.
 * A side effect of debounce is that fast rotations can cause steps to
 * be skipped. By not requiring debounce, fast rotations can be accurately
 * measured.
 * Another advantage is the ability to properly handle bad state, such
 * as due to EMI, etc.
 * It is also a lot simpler than others - a static state table and less
 * than 10 lines of logic.
 */

// #include "Arduino.h"
// #include "Rotary.h"

/*
 * The below state table has, for each state (row), the new state
 * to set based on the next encoder output. From left to right in,
 * the table, the encoder outputs are 00, 01, 10, 11, and the value
 * in that position is the new state to set.
 */

#define R_START 0x0

	// Values returned by 'process'
// No complete step yet.
#define DIR_NONE 0x0
// Clockwise step.
#define DIR_CW 0x10
// Anti-clockwise step.
#define DIR_CCW 0x20

//#define HALF_STEP

#ifdef HALF_STEP
// Use the half-step state table (emits a code at 00 and 11)
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
constexpr uint8_t ttable[6][4] PROGMEM = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
#else
// Use the full-step state table (emits a code at 00 only)
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const uint8_t ttable_P[7][4] PROGMEM  = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif

inline int8_t Encoder::__updateState(PinStatesType pinStates)
{
	auto value = static_cast<uint8_t>(pinStates);
	// reads ttable_P[_state & 0xf][value]
	encoder.state = pgm_read_byte(reinterpret_cast<const uint8_t *>(ttable_P) + (((encoder.state & 0xf) * 4) + value));
	switch(encoder.state) {
		case DIR_CW:
			return 1;
		case DIR_CCW:
			return -1;
	}
	return 0;
}

#endif
