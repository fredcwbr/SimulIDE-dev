#ifndef RAMMEMORYPROXY_H
#define RAMMEMORYPROXY_H

#include <cstdint>

class RamMemoryProxy
{
public:
    virtual ~RamMemoryProxy() = default;

    virtual uint8_t read(uint16_t address) = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;
    virtual void reset() = 0;
};

class DirectRamProxy : public RamMemoryProxy
{
public:
    DirectRamProxy(uint8_t* rawRam = nullptr, uint32_t ramSize = 0)
        : m_ram(rawRam), m_size(ramSize) {}

    void setBuffer(uint8_t* rawRam, uint32_t ramSize)
    {
        m_ram = rawRam;
        m_size = ramSize;
    }

    inline uint8_t read(uint16_t address) override
    {
        if (m_ram && address < m_size) {
            return m_ram[address];
        }
        return 0xFF;
    }

    inline void write(uint16_t address, uint8_t value) override
    {
        if (m_ram && address < m_size) {
            m_ram[address] = value;
        }
    }

    void reset() override {}

private:
    uint8_t* m_ram;
    uint32_t m_size;
};

#endif // RAMMEMORYPROXY_H
       
