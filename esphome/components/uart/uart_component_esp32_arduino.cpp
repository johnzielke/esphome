#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "uart_component_esp32_arduino.h"

#ifdef USE_LOGGER
#include "esphome/components/logger/logger.h"
#endif

namespace esphome
{
  namespace uart
  {
    static const char *const TAG = "uart.arduino_esp32";

    static const uint32_t UART_PARITY_EVEN = 0 << 0;
    static const uint32_t UART_PARITY_ODD = 1 << 0;
    static const uint32_t UART_PARITY_ENABLE = 1 << 1;
    static const uint32_t UART_NB_BIT_5 = 0 << 2;
    static const uint32_t UART_NB_BIT_6 = 1 << 2;
    static const uint32_t UART_NB_BIT_7 = 2 << 2;
    static const uint32_t UART_NB_BIT_8 = 3 << 2;
    static const uint32_t UART_NB_STOP_BIT_1 = 1 << 4;
    static const uint32_t UART_NB_STOP_BIT_2 = 3 << 4;
    static const uint32_t UART_TICK_APB_CLOCK = 1 << 27;

    uint32_t ESP32ArduinoUARTComponent::get_config()
    {
      uint32_t config = 0;

      /*
       * All bits numbers below come from
       * framework-arduinoespressif32/cores/esp32/esp32-hal-uart.h
       * And more specifically conf0 union in uart_dev_t.
       *
       * Below is bit used from conf0 union.
       * <name>:<bits position>  <values>
       * parity:0                0:even 1:odd
       * parity_en:1             Set this bit to enable uart parity check.
       * bit_num:2-4             0:5bits 1:6bits 2:7bits 3:8bits
       * stop_bit_num:4-6        stop bit. 1:1bit  2:1.5bits  3:2bits
       * tick_ref_always_on:27   select the clock.1：apb clock：ref_tick
       */

      if (this->parity_ == UART_CONFIG_PARITY_EVEN)
      {
        config |= UART_PARITY_EVEN | UART_PARITY_ENABLE;
      }
      else if (this->parity_ == UART_CONFIG_PARITY_ODD)
      {
        config |= UART_PARITY_ODD | UART_PARITY_ENABLE;
      }

      switch (this->data_bits_)
      {
      case 5:
        config |= UART_NB_BIT_5;
        break;
      case 6:
        config |= UART_NB_BIT_6;
        break;
      case 7:
        config |= UART_NB_BIT_7;
        break;
      case 8:
        config |= UART_NB_BIT_8;
        break;
      }

      if (this->stop_bits_ == 1)
      {
        config |= UART_NB_STOP_BIT_1;
      }
      else
      {
        config |= UART_NB_STOP_BIT_2;
      }

      config |= UART_TICK_APB_CLOCK;

      return config;
    }

    void ESP32ArduinoUARTComponent::setup()
    {
      ESP_LOGCONFIG(TAG, "Setting up UART...");
      // Use Arduino HardwareSerial UARTs if all used pins match the ones
      // preconfigured by the platform. For example if RX disabled but TX pin
      // is 1 we still want to use Serial.
      bool is_default_tx, is_default_rx;
#ifdef CONFIG_IDF_TARGET_ESP32C3
      is_default_tx = tx_pin_ == nullptr || tx_pin_->get_pin() == 21;
      is_default_rx = rx_pin_ == nullptr || rx_pin_->get_pin() == 20;
#else
      is_default_tx = tx_pin_ == nullptr || tx_pin_->get_pin() == 1;
      is_default_rx = rx_pin_ == nullptr || rx_pin_->get_pin() == 3;
#endif
      static uint8_t next_uart_num = 1;
      if (is_default_tx && is_default_rx)
      {
        this->hw_serial_ = &Serial;
      }
      else
#ifdef ESP32_UART_FALLBACK_TO_SERIAL
       if (next_uart_num < 3)
#endif
      {
        this->hw_serial_ = new HardwareSerial(next_uart_num++); // NOLINT(cppcoreguidelines-owning-memory)
      }
#ifdef ESP32_UART_FALLBACK_TO_SERIAL

      else
      {
        this->sw_serial_ = new ESP32SoftwareSerial();
      }
#endif
      int8_t tx = this->tx_pin_ != nullptr ? this->tx_pin_->get_pin() : -1;
      int8_t rx = this->rx_pin_ != nullptr ? this->rx_pin_->get_pin() : -1;
      bool invert = false;
      if (tx_pin_ != nullptr && tx_pin_->is_inverted())
        invert = true;
      if (rx_pin_ != nullptr && rx_pin_->is_inverted())
        invert = true;
#ifdef ESP32_UART_FALLBACK_TO_SERIAL

      if (this->hw_serial_ != nullptr)
      {
#endif

        this->hw_serial_->begin(this->baud_rate_, get_config(), rx, tx, invert);
        this->hw_serial_->setRxBufferSize(this->rx_buffer_size_);
#ifdef ESP32_UART_FALLBACK_TO_SERIAL

      }
      else
      {
        this->sw_serial_->setup(tx_pin_, rx_pin_, this->baud_rate_, this->stop_bits_, this->data_bits_, this->parity_,
                                this->rx_buffer_size_);
      }
#endif
    }

    void ESP32ArduinoUARTComponent::dump_config()
    {
      ESP_LOGCONFIG(TAG, "UART Bus:");
      LOG_PIN("  TX Pin: ", tx_pin_);
      LOG_PIN("  RX Pin: ", rx_pin_);
      if (this->rx_pin_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  RX Buffer Size: %u", this->rx_buffer_size_); // NOLINT
      }
      ESP_LOGCONFIG(TAG, "  Baud Rate: %u baud", this->baud_rate_);
      ESP_LOGCONFIG(TAG, "  Data Bits: %u", this->data_bits_);
      ESP_LOGCONFIG(TAG, "  Parity: %s", LOG_STR_ARG(parity_to_str(this->parity_)));
      ESP_LOGCONFIG(TAG, "  Stop bits: %u", this->stop_bits_);
      if (this->hw_serial_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Using hardware serial interface.");
      }
      else
      {
        ESP_LOGCONFIG(TAG, "  Using software serial");
      }
      this->check_logger_conflict();
    }

    void ESP32ArduinoUARTComponent::check_logger_conflict()
    {
#ifdef USE_LOGGER
      if (this->hw_serial_ == nullptr || logger::global_logger->get_baud_rate() == 0)
      {
        return;
      }

      if (this->hw_serial_ == logger::global_logger->get_hw_serial())
      {
        ESP_LOGW(TAG, "  You're using the same serial port for logging and the UART component. Please "
                      "disable logging over the serial port by setting logger->baud_rate to 0.");
      }
#endif
    }

    void ESP32ArduinoUARTComponent::write_array(const uint8_t *data, size_t len)
    {
#ifdef ESP32_UART_FALLBACK_TO_SERIAL
      if (this->hw_serial_ != nullptr)
      {
        this->hw_serial_->write(data, len);
      }
      else
      {
        for (size_t i = 0; i < len; i++)
          this->sw_serial_->write_byte(data[i]);
      }
#else
      this->hw_serial_->write(data, len);
#endif
#ifdef USE_UART_DEBUGGER
      for (size_t i = 0; i < len; i++)
      {
        this->debug_callback_.call(UART_DIRECTION_TX, data[i]);
      }
#endif
    }
    bool ESP32ArduinoUARTComponent::peek_byte(uint8_t *data)
    {
      if (!this->check_read_timeout_())
        return false;
#ifdef ESP32_UART_FALLBACK_TO_SERIAL
      if (this->hw_serial_ != nullptr)
      {
        *data = this->hw_serial_->peek();
      }
      else
      {
        *data = this->sw_serial_->peek_byte();
      }
#else
      *data = this->hw_serial_->peek();
#endif
      return true;
    }
    bool ESP32ArduinoUARTComponent::read_array(uint8_t *data, size_t len)
    {
      if (!this->check_read_timeout_(len))
        return false;
#ifdef ESP32_UART_FALLBACK_TO_SERIAL
      if (this->hw_serial_ != nullptr)
      {
        this->hw_serial_->readBytes(data, len);
      }
      else
      {
        for (size_t i = 0; i < len; i++)
          data[i] = this->sw_serial_->read_byte();
      }
#else
      this->hw_serial_->readBytes(data, len);
#endif
#ifdef USE_UART_DEBUGGER
      for (size_t i = 0; i < len; i++)
      {
        this->debug_callback_.call(UART_DIRECTION_RX, data[i]);
      }
#endif
      return true;
    }
    int ESP32ArduinoUARTComponent::available()
    {
#ifdef ESP32_UART_FALLBACK_TO_SERIAL
      if (this->hw_serial_ != nullptr)
      {
        return this->hw_serial_->available();
      }
      else
      {
        return this->sw_serial_->available();
      }
#else
          return this->hw_serial_->available();
#endif // ESP32_UART_FALLBACK_TO_SERIAL
    }
    void ESP32ArduinoUARTComponent::flush()
    {
      ESP_LOGVV(TAG, "    Flushing...");
#ifdef ESP32_UART_FALLBACK_TO_SERIAL

      if (this->hw_serial_ != nullptr)
      {
        this->hw_serial_->flush();
      }
      else
      {
        this->sw_serial_->flush();
      }
#else
      this->hw_serial_->flush();
#endif // ESP32_UART_FALLBACK_TO_SERIAL
    }

#ifdef ESP32_UART_FALLBACK_TO_SERIAL
    void ESP32SoftwareSerial::setup(InternalGPIOPin *tx_pin, InternalGPIOPin *rx_pin, uint32_t baud_rate,
                                      uint8_t stop_bits, uint32_t data_bits, UARTParityOptions parity,
                                      size_t rx_buffer_size)
    {
      this->bit_time_ = F_CPU / baud_rate;
      this->rx_buffer_size_ = rx_buffer_size;
      this->stop_bits_ = stop_bits;
      this->data_bits_ = data_bits;
      this->parity_ = parity;
      if (tx_pin != nullptr)
      {
        gpio_tx_pin_ = tx_pin;
        gpio_tx_pin_->setup();
        tx_pin_ = gpio_tx_pin_->to_isr();
        tx_pin_.digital_write(true);
      }
      if (rx_pin != nullptr)
      {
        gpio_rx_pin_ = rx_pin;
        gpio_rx_pin_->setup();
        rx_pin_ = gpio_rx_pin_->to_isr();
        rx_buffer_ = new uint8_t[this->rx_buffer_size_]; // NOLINT
        gpio_rx_pin_->attach_interrupt(ESP32SoftwareSerial::gpio_intr, this, gpio::INTERRUPT_FALLING_EDGE);
      }
    }
    void IRAM_ATTR ESP32SoftwareSerial::gpio_intr(ESP32SoftwareSerial *arg)
    {
      uint32_t wait = arg->bit_time_ + arg->bit_time_ / 3 - 500;
      const uint32_t start = arch_get_cpu_cycle_count();
      uint8_t rec = 0;
      // Manually unroll the loop
      for (int i = 0; i < arg->data_bits_; i++)
        rec |= arg->read_bit_(&wait, start) << i;

      /* If parity is enabled, just read it and ignore it. */
      /* TODO: Should we check parity? Or is it too slow for nothing added..*/
      if (arg->parity_ == UART_CONFIG_PARITY_EVEN || arg->parity_ == UART_CONFIG_PARITY_ODD)
        arg->read_bit_(&wait, start);

      // Stop bit
      arg->wait_(&wait, start);
      if (arg->stop_bits_ == 2)
        arg->wait_(&wait, start);

      arg->rx_buffer_[arg->rx_in_pos_] = rec;
      arg->rx_in_pos_ = (arg->rx_in_pos_ + 1) % arg->rx_buffer_size_;
      // Clear RX pin so that the interrupt doesn't re-trigger right away again.
      arg->rx_pin_.clear_interrupt();
    }
    void IRAM_ATTR HOT ESP32SoftwareSerial::write_byte(uint8_t data)
    {
      if (this->gpio_tx_pin_ == nullptr)
      {
        ESP_LOGE(TAG, "UART doesn't have TX pins set!");
        return;
      }
      bool parity_bit = false;
      bool need_parity_bit = true;
      if (this->parity_ == UART_CONFIG_PARITY_EVEN)
      {
        parity_bit = false;
      }
      else if (this->parity_ == UART_CONFIG_PARITY_ODD)
      {
        parity_bit = true;
      }
      else
      {
        need_parity_bit = false;
      }

      {
        InterruptLock lock;
        uint32_t wait = this->bit_time_;
        const uint32_t start = arch_get_cpu_cycle_count();
        // Start bit
        this->write_bit_(false, &wait, start);
        for (int i = 0; i < this->data_bits_; i++)
        {
          bool bit = data & (1 << i);
          this->write_bit_(bit, &wait, start);
          if (need_parity_bit)
            parity_bit ^= bit;
        }
        if (need_parity_bit)
          this->write_bit_(parity_bit, &wait, start);
        // Stop bit
        this->write_bit_(true, &wait, start);
        if (this->stop_bits_ == 2)
          this->wait_(&wait, start);
      }
    }
    void IRAM_ATTR ESP32SoftwareSerial::wait_(uint32_t *wait, const uint32_t &start)
    {
      while (arch_get_cpu_cycle_count() - start < *wait)
        ;
      *wait += this->bit_time_;
    }
    bool IRAM_ATTR ESP32SoftwareSerial::read_bit_(uint32_t *wait, const uint32_t &start)
    {
      this->wait_(wait, start);
      return this->rx_pin_.digital_read();
    }
    void IRAM_ATTR ESP32SoftwareSerial::write_bit_(bool bit, uint32_t *wait, const uint32_t &start)
    {
      this->tx_pin_.digital_write(bit);
      this->wait_(wait, start);
    }
    uint8_t ESP32SoftwareSerial::read_byte()
    {
      if (this->rx_in_pos_ == this->rx_out_pos_)
        return 0;
      uint8_t data = this->rx_buffer_[this->rx_out_pos_];
      this->rx_out_pos_ = (this->rx_out_pos_ + 1) % this->rx_buffer_size_;
      return data;
    }
    uint8_t ESP32SoftwareSerial::peek_byte()
    {
      if (this->rx_in_pos_ == this->rx_out_pos_)
        return 0;
      return this->rx_buffer_[this->rx_out_pos_];
    }
    void ESP32SoftwareSerial::flush()
    {
      // Flush is a NO-OP with software serial, all bytes are written immediately.
    }
    int ESP32SoftwareSerial::available()
    {
      int avail = int(this->rx_in_pos_) - int(this->rx_out_pos_);
      if (avail < 0)
        return avail + this->rx_buffer_size_;
      return avail;
    }
#endif // ESP32_UART_FALLBACK_TO_SERIAL

  } // namespace uart
} // namespace esphome
#endif // USE_ESP32_FRAMEWORK_ARDUINO
