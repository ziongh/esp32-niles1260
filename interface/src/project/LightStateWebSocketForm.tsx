import { FC, useState } from 'react';

import { Stack, Slider } from '@mui/material';
import VolumeDown from '@mui/icons-material/VolumeDown';
import VolumeUp from '@mui/icons-material/VolumeUp';

import { WEB_SOCKET_ROOT } from '../api/endpoints';
import { BlockFormControlLabel, FormLoader, MessageBox, SectionContent } from '../components';
import { updateValueDirect, useWs } from '../utils';

import { VolumeState } from './types';

export const VOLUME_SETTINGS_WEBSOCKET_URL = WEB_SOCKET_ROOT + "volumeState";

const LightStateWebSocketForm: FC = () => {
  const { connected, updateData, data } = useWs<VolumeState>(VOLUME_SETTINGS_WEBSOCKET_URL);

  const updateFormValue = updateValueDirect(updateData);

  const content = () => {
    if (!connected || !data) {
      return (<FormLoader message="Connecting to WebSocket…" />);
    }
    return (
      <>
        <BlockFormControlLabel
          style={{width: '100%' }}
          control={
            <Stack style={{width: '100%' }} spacing={2} direction="row" sx={{ mb: 1 }} alignItems="center">
              <VolumeDown />
              <Slider
                name="volumeSala"
                aria-label="volumeSala"
                max={100}
                min={0}
                style={{width: '100%' }}
                onChange={(e, value) => updateFormValue(value as  number, 'volumeSala')}
              />
              <VolumeUp />
            </Stack>
          }
          label="Volume Sala"
          labelPlacement='top'
        />
        <BlockFormControlLabel
          style={{width: '100%' }}
          control={
            <Stack style={{width: '100%' }} spacing={2} direction="row" sx={{ mb: 1 }} alignItems="center">
              <VolumeDown />
              <Slider
                name="volumeCinema"
                aria-label="volumeCinema"
                max={100}
                min={0}
                style={{width: '100%' }}
                onChange={(e, value) => updateFormValue(value as  number, 'volumeCinema')}
              />
              <VolumeUp />
            </Stack>
          }
          label="Volume Cinema"
          labelPlacement='top'
        />
        <BlockFormControlLabel
          style={{width: '100%' }}
          control={
            <Stack style={{width: '100%' }} spacing={2} direction="row" sx={{ mb: 1 }} alignItems="center">
              <VolumeDown />
              <Slider
                name="volumeVaranda"
                aria-label="volumeVaranda"
                max={100}
                min={0}
                style={{width: '100%' }}
                onChange={(e, value) => updateFormValue(value as  number, 'volumeVaranda')}
              />
              <VolumeUp />
            </Stack>
          }
          label="Volume Varanda"
          labelPlacement='top'
        />
        <BlockFormControlLabel
          style={{width: '100%' }}
          control={
            <Stack style={{width: '100%' }} spacing={2} direction="row" sx={{ mb: 1 }} alignItems="center">
              <VolumeDown />
              <Slider
                name="volumeCozinha"
                aria-label="volumeCozinha"
                max={100}
                min={0}
                style={{width: '100%' }}
                onChange={(e, value) => updateFormValue(value as  number, 'volumeCozinha')}
              />
              <VolumeUp />
            </Stack>
          }
          label="Volume Cozinha"
          labelPlacement='top'
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
