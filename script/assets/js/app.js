import { define, html, dispatch } from 'https://esm.sh/hybrids@^8';

function instanceWrapper(klass, property) {
    return (fn) => {
        return (host, target) => {
            fn(host, target);

            const inst = new klass(host);

            if(property !== undefined) {
                host[property] = inst;
            }
        }
    }
}

define({
    tag: "x-chart-figure",
    graphValues: undefined,
    title: undefined,
    sensorType: 'high_resolution_filament_sensor',
    content: () => html`
        <figure class="highcharts-figure">
            <div name="container">
                <div name="details"></div>
                <div name="timeline"></div>
            </div>
        </figure>
        `.css`
        figure div[name=container] {
            position: relative;
            height: 400px;
        }
        figure div[name=timeline] {
            position: absolute;
            top: 300px;
            height: 100px;
            width: 100%;
        }
        `
        .use(instanceWrapper(TimeseriesChart, 'chart')),
});

define({
    tag: "x-charts",
    content: () =>
        html`
        <x-chart-figure title="Temperature" graph-values="CurrentTemperature,TargetTemperature" sensor-type="extruder"></x-chart-figure>
        <x-chart-figure title="Underextrusion" graph-values="Underextrusion"></x-chart-figure>
        <x-chart-figure title="Measured vs. expected extrusion distance" graph-values="MeasuredDistance,ExpectedDistance,CommandedDistance"></x-chart-figure>
        <x-chart-figure title="Measured speed" graph-values="MeasuredSpeed"></x-chart-figure>
        <x-chart-figure title="Measured volumetric flow" graph-values="MeasuredVolumetricFlow"></x-chart-figure>
        `
        .use(instanceWrapper(Charts)),
});

define({
    tag: "x-charts-options",
    content: () =>
        html`
        options!
        `,
});

class Sensor {
    constructor(name) {
        this.name = name;
        var nameParts = name.split(' ');
        this.type = nameParts[0];
        if(this.type.match(/^extruder\d+$/))
            this.type = 'extruder';
        this.label = nameParts[nameParts.length - 1];
        this.timeline = {}
        this.status = null;
    }

    add(data, eventtime) {
        if(this.status === null) {
            this.status = {...data};
        } else {
            this.status = this.mergeDeep(this.status, data);
        }
        this.timeline[eventtime] = this.status;
    }

    get(eventtime) {
        if(eventtime in this.timeline)
            return this.timeline[eventtime];

        let closest;
        for(const key in this.timeline) {
            if(parseFloat(key) > eventtime) {
                if(!closest) closest = this.timeline[key];
                break;
            }
            closest = this.timeline[key];
        }

        return closest;
    }

    isObject(item) {
        return (item && typeof item === 'object' && !Array.isArray(item));
    }

    mergeDeep(target, source) {
        let output = Object.assign({}, target);
        if (this.isObject(target) && this.isObject(source)) {
          Object.keys(source).forEach(key => {
            if (this.isObject(source[key])) {
              if (!(key in target))
                Object.assign(output, { [key]: source[key] });
              else
                output[key] = this.mergeDeep(target[key], source[key]);
            } else {
              Object.assign(output, { [key]: source[key] });
            }
          });
        }
        return output;
    }
}

class Events {
    constructor() {
        this._triggers = {};
    }

    on(event, callback) {
        if(!this._triggers[event])
            this._triggers[event] = [];
        this._triggers[event].push(callback);
    }

    triggerHandler(event, params) {
        if(this._triggers[event]) {
            for(const i in this._triggers[event])
                this._triggers[event][i](params);
        }
    }
};

class DataReceiver {
    constructor() {
        this.socket = new WebSocket("ws://" + document.location.hostname + ":" + document.location.port + "/api/ws")
        this.socket.onopen = (event) => { this.onopen(event); };
        this.socket.onmessage = (event) => { this.onmessage(event); };

        this.sensors = {};
        this.events = new Events();
    }

    onopen(event) {
        console.log('Websocket connected');
    }

    onmessage(event) {
        const msg = JSON.parse(event.data);
        if('now' in msg) {
            this.events.triggerHandler('time-received', {now: msg.now});
            return;
        }
        const type = msg.type;
        let eventtimes = [];
        for (var key in msg.events) {
            const eventtime = parseFloat(key);
            eventtimes.push(eventtime);
            const statuses = msg.events[key];
            for (var name in statuses) {
                var data = statuses[name];
                if (!(name in this.sensors)) {
                    this.sensors[name] = new Sensor(name);
                }

                const sensor = this.sensors[name];
                sensor.add(data, eventtime);

                // trigger callback each time an event is added to a sensor timeline
                this.events.triggerHandler('sensor-timeline-update', {type, sensor, eventtime});
            }
        }

        // trigger callback after a single event or a batch of events has been processed
        this.events.triggerHandler('events-received', {type, events: eventtimes});
    }
}

class GraphSensorValue {
    static Underextrusion = new GraphSensorValue('Underextrusion', '%');
    static MeasuredSpeed = new GraphSensorValue('MeasuredSpeed', 'mm/s');
    static MeasuredVolumetricFlow = new GraphSensorValue('MeasuredVolumetricFlow', 'mm³/s');
    static MeasuredDistance = new GraphSensorValue('MeasuredDistance', 'mm');
    static ExpectedDistance = new GraphSensorValue('ExpectedDistance', 'mm');
    static CommandedDistance = new GraphSensorValue('CommandedDistance', 'mm');
    static CurrentTemperature = new GraphSensorValue('CurrentTemperature', '°C');
    static TargetTemperature = new GraphSensorValue('TargetTemperature', '°C');

    constructor(name, unit) {
        this.name = name;
        this.unit = unit;
    }

    fromSensorData(data) {
        switch(this.name) {
            case 'Underextrusion':
                return data['underextrusion_rate'] * 100;
            case 'MeasuredSpeed':
                return data['motion']['measured_speed'];
            case 'MeasuredVolumetricFlow':
                return data['motion']['measured_volumetric_flow'];
            case 'MeasuredDistance':
                return data['motion']['measured_distance'];
            case 'ExpectedDistance':
                return data['motion']['expected_distance'];
            case 'CommandedDistance':
                return data['motion']['commanded_distance'];
            case 'CurrentTemperature':
                return data['temperature'];
            case 'TargetTemperature':
                return data['target'];
        }
        return;
    }

    toString() {
        return `GraphValue.${this.name}`;
    }
}

class Charts {
    constructor(host) {
        this.host = host;
        this.receiver = new DataReceiver();

        this.lastEventtime = undefined;
        this.timeOffset = 0.0;

        this.receiver.events.on('sensor-timeline-update', (event) => {
            if (event.type == 'live') {
                this.liveUpdate(event);
            }
        });

        this.receiver.events.on('events-received', (event) => {
            if(event.type == 'historical') {
                this.historicalBatchUpdate(event);
            }
        });

        this.receiver.events.on('time-received', ({ now }) => {
            const offset = now - (new Date().getTime() / 1000);
            console.log(['received time', now, '=>', new Date().getTime() / 1000, 'offset:', offset]);
            this.timeOffset = offset;
            for(const chart of this.charts()) {
                chart.timeOffset = offset;
            }
        });

        setTimeout(this.injectUpdate.bind(this), 1000);

        for(const el of this.chartsElements()) {
            el.addEventListener("chart-selection", ({ detail }) => {
                this.selectionEvent(el, detail);
            });

            el.addEventListener('mousemove', ((e) => this.mouseEvent(el, e)).bind(this));
            el.addEventListener('touchmove', ((e) => this.mouseEvent(el, e)).bind(this));
            el.addEventListener('touchstart', ((e) => this.mouseEvent(el, e)).bind(this));
        }
    }

    injectUpdate() {
        const localTime = new Date().getTime() / 1000;
        const lastEvent = this.lastEventtime + this.timeOffset;
        const timeSinceUpdate = localTime - lastEvent;

        if(timeSinceUpdate > 5) {
            const eventtime = localTime - this.timeOffset;
            console.log('not received update for', timeSinceUpdate);
            this.lastEventtime = eventtime;
            this.liveUpdate({eventtime: eventtime});
        }

        setTimeout(this.injectUpdate.bind(this), 100);
    }

    chartsElements() {
        return this.host.querySelectorAll("x-chart-figure");
    }

    charts() {
        return [...this.chartsElements()].map((el) => el.chart);
    }

    liveUpdate(event) {
        // console.log(['sensor timeline update', event.type, event.sensor.name, event.eventtime]);
        for(const chart of this.charts()) {
            // chart.addSensorData(event.sensor, [event.eventtime], true);
            chart.addMultiSensorData(Object.values(this.receiver.sensors), [event.eventtime]);
        }
        this.lastEventtime = event.eventtime;
        this.syncRanges();
    }

    historicalBatchUpdate(event) {
        // console.log(['received batched events', event.type, event.events]);
        for(const chart of this.charts()) {
            chart.addMultiSensorData(Object.values(this.receiver.sensors), event.events);
        }
        this.lastEventtime = event.events[event.events.length - 1];
        this.syncRanges();
    }

    syncRanges() {
        let min, max;
        for(const chart of this.charts()) {
            const range = chart.timelineRange();
            if(range.min && (!min || range.min < min)) {
                min = range.min;
            }
            if(range.max && (!max || range.max > max)) {
                max = range.max;
            }
        }

        // console.log(['current timeline', min, max]);

        if(min && max) {
            for(const chart of this.charts()) {
                chart.setTimelineRange(min, max);
            }
        }
    }

    selectionEvent(source, event) {
        for(const chart of this.charts()) {
            chart.setDetailsRange(event.xAxis[0].min, event.xAxis[0].max);
        }
    }

    mouseEvent(source, e) {
        const charts = this.charts();
        const event = source.chart.details.pointer.normalize(e);
        for (let i = 0; i < charts.length; i = i + 1) {
            const chart = charts[i].details;
            if(chart.series.length == 0)
                continue;

            const point = chart.series[0].searchPoint(event, true);

            if (point) {
                point.onMouseOver(); // Show the hover marker
                chart.tooltip.refresh(this); // Show the tooltip
                // chart.xAxis[0].drawCrosshair(event, point); // Show the crosshair
            }
        }
    }
}

class TimeseriesChart {
    constructor(host) {
        this.host = host;

        this.graphValues = this.host.graphValues.split(',').map((v) => GraphSensorValue[v]);
        this.unit = this.graphValues[0].unit;
        this.details = this.createDetails();
        this.timeline = this.createTimeline();
        this.maskFill = 'rgba(0,0,255,0.05)';

        this.started = new Date().getTime();
        this.timeOffset = null;
        this.seriesNums = {};
        this.seriesData = {};

        this.addLine(this.started, 'Start');
        this.details.setTitle({text: this.host.title});

        this.sensorType = this.host.sensorType;

        this.currentLayerChangedAt = null;
        this.currentLayerNumber = null;

        this.visibleRange = {
            min: null,
            max: null,
        };
        this.timelineExtremes = {
            min: this.started,
            max: this.started + (60 * 1000),
        }
        this.setTimelineRange(this.timelineExtremes.min, this.timelineExtremes.max);
    }

    addLine(x, text) {
        this.timeline.xAxis[0].addPlotLine({
            color: '#FF0000', // Red
            width: 0.1,
            value: x,
        });
        this.details.xAxis[0].addPlotLine({
            color: '#FF0000', // Red
            width: 0.1,
            value: x,
            label: {text},
        });
    }

    addMultiSensorData(sensors, eventtimes) {
        for (const sensor of sensors) {
            this.addSensorData(sensor, eventtimes, false);
        }
        this.timeline.redraw(false);
        this.details.redraw(false);
    }

    addSensorData(sensor, eventtimes, redraw) {
        if(sensor.type == 'print_stats') {
            for(const eventtime of eventtimes) {
                const sensorData = sensor.get(eventtime);
                if(!sensorData) continue;

                const cur = sensorData['info']['current_layer'],
                        total = sensorData['info']['total_layer'];

                if(cur !== undefined && cur !== null && cur != this.currentLayerNumber) {
                    console.log(['print_stats', eventtime, sensorData]);
                    this.currentLayerNumber = cur;
                    this.currentLayerChangedAt = eventtime;
                    console.log(`Layer changed: ${cur}/${total}`);
                }
            }
            return;
        }

        if(sensor.type != this.sensorType)
            return;

        for(const graphValue of this.graphValues) {
            const seriesNum = this.getSeriesNum(sensor, graphValue);
            const timelineSeries = this.timeline.series[seriesNum];
            const detailsSeries = this.details.series[seriesNum];

            for(const eventtime of eventtimes) {
                const x = this.printerTimeToChartTime(eventtime);
                const sensorData = sensor.get(eventtime);

                if(sensorData) {
                    const y = graphValue.fromSensorData(sensorData);

                    if(y === undefined || y === null) {
                        continue;
                    }

                    timelineSeries.addPoint([x, y], false, false);

                    // if((this.visibleRange.min === null || x > this.visibleRange.min) &&
                    //     (this.visibleRange.max === null || x < this.visibleRange.max)) {
                        detailsSeries.addPoint([x, y], false, false);
                    // }

                    let min = this.timelineExtremes.min, max = this.timelineExtremes.min;
                    if(min === null || x < min) min = x;
                    if(max === null || x > max) max = x;
                    this.timelineExtremes.min = min;
                    this.timelineExtremes.max = max;
                }
                else {
                    // console.log('empty sensor data', sensor, eventtime);
                }
            }
        }

        if(redraw) {
            this.timeline.redraw(false);
            this.details.redraw(false);
        }
    }

    printerTimeToChartTime(eventtime) {
        // if(this.timeOffset === null) {
        //     this.timeOffset = eventtime;
        // }
        // return this.started + ((eventtime - this.timeOffset) * 1000);
        return ((this.timeOffset + eventtime) * 1000);
    }

    getSeriesNum(sensor, graphValue) {
        const key = `${sensor.name}.${graphValue.name}`;
        if (this.seriesNums[key] === undefined) {
            const seriesNum = this.timeline.series.length;

            this.seriesData[seriesNum] = [];

            this.timeline.addSeries({
                name: `${sensor.label}.${graphValue.name}`,
                pointInterval: 1000,
                pointStart: this.started,
                data: this.seriesData[seriesNum],
            });
            this.details.addSeries({
                name: `${sensor.label}.${graphValue.name}`,
                pointInterval: 100,
                pointStart: this.started,
                data: this.seriesData[seriesNum],
            });

            this.seriesNums[key] = seriesNum;
        }

        return this.seriesNums[key];
    }

    timelineRange() {
        // return {min: this.timeline.xAxis[0].min, max: this.timeline.xAxis[0].max};
        return this.timelineExtremes;
    }

    setTimelineRange(min, max) {
        if(this.timeline.xAxis[0].min != min || this.timeline.xAxis[0].max != max) {
            this.timeline.xAxis[0].setExtremes(min, max);
            this.redrawPlotBands();
        }
    }

    setDetailsRange(min, max) {
        this.visibleRange.min = min;

        // console.log(['select max:', max, 'axis max:', this.timeline.xAxis[0].max]);
        if(max >= this.timeline.xAxis[0].max)
            this.visibleRange.max = null;
        else
            this.visibleRange.max = max;

        this.details.xAxis[0].setExtremes(this.visibleRange.min, this.visibleRange.max);

        this.redrawPlotBands();
    }

    redrawPlotBands() {
        const xAxis = this.timeline.xAxis[0];

        xAxis.removePlotBand('mask-before');
        if(this.visibleRange.min) {
            xAxis.addPlotBand({
                id: 'mask-before',
                from: xAxis.min,
                to: this.visibleRange.min,
                color: this.maskFill
            });
        }

        xAxis.removePlotBand('mask-after');
        if(this.visibleRange.max) {
            xAxis.addPlotBand({
                id: 'mask-after',
                from: this.visibleRange.max,
                to: xAxis.max,
                color: this.maskFill
            });
        }
    }

    selection(event) {
        dispatch(this.host, 'chart-selection', { detail: event });
        return false;
    }

    createDetails() {
        // create a detail chart referenced by a global variable
        return Highcharts.chart(this.host.querySelectorAll("[name=details]")[0], {
            chart: {
                marginBottom: 120,
                reflow: false,
                marginLeft: 50,
                marginRight: 20,
                style: {
                    position: 'absolute'
                },
                zoomType: 'x',
                events: {
                    selection: this.selection.bind(this),
                },
            },
            credits: {
                enabled: false
            },
            title: {
                text: '',
                align: 'left'
            },
            xAxis: {
                type: 'datetime',
                maxZoom: 1000,
                crosshair: true,
            },
            yAxis: {
                title: {
                    text: this.unit,
                },
                maxZoom: 0.1,
            },
            // tooltip: {
            //     format: '<b>{series.name}</b><br/>{x:%A %B %e %Y}:<br/>' +
            //             '{y:.2f}',
            //     shared: true
            // },
            tooltip: {
                // positioner: function () {
                //     return {
                //         // right aligned
                //         x: this.chart.chartWidth - this.label.width,
                //         y: 10 // align to title
                //     };
                // },
                borderWidth: 0.1,
                // backgroundColor: 'none',
                pointFormat: `{series.name}: {point.y} ${this.unit}<br />`,
                headerFormat: '',
                // shadow: false,
                style: {
                    fontSize: '12px'
                },
                valueDecimals: 2,
                shared: true,
            },
            legend: {
                enabled: false
            },
            plotOptions: {
                series: {
                    marker: {
                        enabled: false,
                        states: {
                            hover: {
                                enabled: true,
                                radius: 3
                            }
                        }
                    }
                }
            },
            series: [],
            exporting: {
                enabled: false
            }
        });
    }

    createTimeline() {
        return Highcharts.chart(this.host.querySelectorAll("[name=timeline]")[0], {
            chart: {
                reflow: false,
                borderWidth: 0,
                backgroundColor: null,
                marginLeft: 50,
                marginRight: 20,
                zoomType: 'x',
                events: {
                    selection: this.selection.bind(this),
                }
            },
            title: {
                text: null
            },
            accessibility: {
                enabled: false
            },
            xAxis: {
                type: 'datetime',
                showLastTickLabel: true,
                maxZoom: 1000 * 60,
                plotBands: [
                    // {
                    //     id: 'mask-before',
                    //     from: chartLoadTime,
                    //     to: chartLoadTime + (1000 * 60),
                    //     color: maskFill
                    // }
                ],
                title: {
                    text: null
                }
            },
            yAxis: {
                gridLineWidth: 0,
                labels: {
                    enabled: false
                },
                title: {
                    text: null
                },
                showFirstLabel: false
            },
            legend: {
                enabled: false
            },
            credits: {
                enabled: false
            },
            plotOptions: {
                series: {
                    fillColor: {
                        linearGradient: [0, 0, 0, 70],
                        stops: [
                            [0, Highcharts.getOptions().colors[0]],
                            [1, 'rgba(255,255,255,0)']
                        ]
                    },
                    lineWidth: 1,
                    marker: {
                        enabled: false
                    },
                    shadow: false,
                    states: {
                        hover: {
                            lineWidth: 1
                        }
                    },
                    enableMouseTracking: false
                }
            },

            series: [],

            exporting: {
                enabled: false
            }

        });
    }
}

class App {
    constructor() {

    }
}

var ready = function(_callback) {
    var callback = function() { setTimeout(_callback, 1); }
    if (document.readyState === "interactive" || document.readyState === "complete") {
        callback();
    } else if (document.addEventListener) {
        document.addEventListener("DOMContentLoaded", callback);
    } else if (document.attachEvent) {
        document.attachEvent("onreadystatechange", function() {
            if (document.readyState != "loading") {
                callback();
            }
        });
    }
};

ready(function() {
    const app = new App();
    window.app = app;
});
