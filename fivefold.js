import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['Design Light Five Fold'],
    model: 'Design Light Five Fold',
    vendor: 'Espressif',
    description: '5 DALI dimmers',
    extend: [
        m.light(),
        m.enumLookup({
            name: "effect_type",
            lookup: { OFF: 0, ON: 1, FADING: 2, FADE_SYNC: 3, LIGHT_RAIL: 4, TWINKLE: 5, SWOOSH: 6 },
            cluster: "genMultistateValue",
            attribute: "presentValue",
            zigbeeCommandOptions: {},
            description: "Effect type",
            access: "ALL",
            reporting: null,
        }),
    ],
    meta: {},
};