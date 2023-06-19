import { FC } from 'react';

import { Stack, Slider } from '@mui/material';
import VolumeDown from '@mui/icons-material/VolumeDown';
import VolumeUp from '@mui/icons-material/VolumeUp';

import { WEB_SOCKET_ROOT } from '../api/endpoints';
import { BlockFormControlLabel, FormLoader, MessageBox, SectionContent } from '../components';
import { updateValueDirect, useWs } from '../utils';

import { VolumeState } from './types';

export const LIGHT_SETTINGS_WEBSOCKET_URL = WEB_SOCKET_ROOT + "volumeState";

const LightStateWebSocketForm: FC = () => {
  const { connected, updateData, data } = useWs<VolumeState>(LIGHT_SETTINGS_WEBSOCKET_URL);

  const updateFormValue = updateValueDirect(updateData);

  const content = () => {
    if (!connected || !data) {
      return (<FormLoader message="Connecting to WebSocket…" />);
    }
    return (
      <>
        <MessageBox
          level="info"
          message="The switch below controls the LED via the WebSocket. It will automatically update whenever the LED state changes."
          my={2}
        />
        <BlockFormControlLabel
          control={
            <Stack spacing={2} direction="row" sx={{ mb: 1 }} alignItems="center">
              <VolumeDown />
              <Slider
                name="volume"
                aria-label="Volume"
                value={data.volume}
                max={100}
                min={0}
                onChange={(e, value) => updateFormValue(value as number, 'volume')}
              />
              <VolumeUp />
            </Stack>
          }
          label="Volume"
        />
      </>
    );
  };

  return (
    <SectionContent title='WebSocket Example' titleGutter>
      {content()}
    </SectionContent>
  );
};

export default LightStateWebSocketForm;
